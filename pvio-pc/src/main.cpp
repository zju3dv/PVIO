/**************************************************************************
* This file is part of PVIO
*
* Copyright (c) ZJU-SenseTime Joint Lab of 3D Vision. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/
#include <chrono>
#include <iostream>
#include <thread>
#include <nanogui/nanogui.h>
#include <nanovis/nanovis.h>
#include <pvio/extra/opencv_image.h>
#include <pvio/extra/yaml_config.h>
#include <pvio/forensics.h>
#include <pvio/pvio.h>
#include <dataset_reader.h>
#include <output_writer.h>

#define TARGET_FPS (30)
#define glsl_code(str) "#version 330\n" #str

class OpenCvPainter : public pvio::ForensicsPainter {
    cv::Mat &canvas;

  public:
    OpenCvPainter(cv::Mat &canvas) :
        canvas(canvas) {
    }

    void set_image(const pvio::Image *image) override {
        const pvio::extra::OpenCvImage *cvimage = dynamic_cast<const pvio::extra::OpenCvImage *>(image);
        cv::cvtColor(cvimage->image, canvas, cv::COLOR_GRAY2BGR);
    }

    void point(const pvio::point2i &p, const pvio::color3b &c, int size = 1, int style = 0) override {
        cv::Point center(p.x(), p.y());
        cv::Scalar color = CV_RGB(c.x(), c.y(), c.z());
        switch (style) {
        case 0: // .
        {
            cv::circle(canvas, center, size, color, -1);
            break;
        }
        case 1: // o
        {
            cv::circle(canvas, center, size, color, 1);
            break;
        }
        case 2: // +
        {
            cv::Point dx(size, 0);
            cv::Point dy(0, size);
            cv::line(canvas, center + dx, center - dx, color);
            cv::line(canvas, center + dy, center - dy, color);
            break;
        }
        case 3: // x
        {
            cv::Point du(size, size);
            cv::Point dv(size, -size);
            cv::line(canvas, center + du, center - du, color);
            cv::line(canvas, center + dv, center - dv, color);
            break;
        }
        }
    }

    void line(const pvio::point2i &p1, const pvio::point2i &p2, const pvio::color3b &c, int thickness = 1) override {
        cv::Point point1(p1.x(), p1.y());
        cv::Point point2(p2.x(), p2.y());
        cv::Scalar color = CV_RGB(c.x(), c.y(), c.z());
        cv::line(canvas, point1, point2, color, thickness);
    }
};

class PVIOPcVisualizer : public nanovis::NanoVis {
    std::unique_ptr<DatasetReader> dataset_reader;
    std::shared_ptr<pvio::Config> pvio_config;
    std::unique_ptr<pvio::PVIO> pvio_instance;

    cv::Mat feature_tracker_image;
    std::unique_ptr<pvio::ForensicsPainter> feature_tracker_painter;

    cv::Mat sliding_window_track_image;
    std::unique_ptr<pvio::ForensicsPainter> sliding_window_track_painter;

    cv::Mat sliding_window_reprojection_image;
    std::unique_ptr<pvio::ForensicsPainter> sliding_window_reprojection_painter;

    double input_data_fps;
    double input_real_fps;
    double input_output_lag;
    double feature_tracker_time;
    double bundle_adjustor_solve_time;

    pvio::quaternion latest_output_q;
    pvio::vector<3> latest_output_p;
    pvio::quaternion latest_sliding_window_q;
    pvio::vector<3> latest_sliding_window_p;
    pvio::vector<3> latest_sliding_window_bg;
    pvio::vector<3> latest_sliding_window_ba;
    std::vector<pvio::quaternion> sliding_window_keyframe_qs;
    std::vector<pvio::vector<3>> sliding_window_keyframe_ps;

    std::vector<pvio::vector<3>> landmark_points;
    std::vector<pvio::vector<3>> landmark_colors;

    std::vector<pvio::OutputPlane> landmark_planes;

    std::vector<pvio::vector<3>> sliding_window_trajectory;
    std::vector<pvio::vector<3>> output_trajectory;

    nanogui::GLShader shader;
    pvio::matrix<3> K;

    std::unique_ptr<OutputWriter> output_writer;

  public:
    PVIOPcVisualizer(const std::string &data_path, const std::string &yaml_path) {
        // [1] data
        dataset_reader = DatasetReader::create_reader(data_path);
        if (!dataset_reader) {
            std::cout << "Cannot open file " << data_path << std::endl;
            exit(EXIT_FAILURE);
        }

        // [2] config
        pvio_config = std::make_shared<pvio::extra::YamlConfig>(yaml_path);

        // [3] algorithm
        pvio_instance = std::make_unique<pvio::PVIO>(pvio_config);

        // [4] gui
        feature_tracker_painter = std::make_unique<OpenCvPainter>(feature_tracker_image);
        forensics(feature_tracker_painter, painter) {
            painter = feature_tracker_painter.get();
        }
        add_image("Visualize", "Feature Tracker Image", feature_tracker_image);

        sliding_window_track_painter = std::make_unique<OpenCvPainter>(sliding_window_track_image);
        forensics(sliding_window_track_painter, painter) {
            painter = sliding_window_track_painter.get();
        }
        add_image("Visualize", "Sliding Window Track Image", sliding_window_track_image);

        sliding_window_reprojection_painter = std::make_unique<OpenCvPainter>(sliding_window_reprojection_image);
        forensics(sliding_window_reprojection_painter, painter) {
            painter = sliding_window_reprojection_painter.get();
        }
        add_image("Visualize", "Sliding Window Reprojection Image", sliding_window_reprojection_image);

        add_graph("Control", "Data FPS", input_data_fps, 60.0, 0);
        add_graph("Control", "Real FPS", input_real_fps, 60.0, 0);
        add_graph("Control", "I/O Lag", input_output_lag, 500.0, 0);
        add_graph("Control", "FT Time", feature_tracker_time, 50.0, 0);
        add_graph("Control", "BA Time", bundle_adjustor_solve_time, 100.0, 0);
        add_button("Control", "Step", [this] { step(); });
        add_repeat("Control", "Run", [this] { return step(); });
        add_button("Control", "Shutdown", [this] { return shutdown(); });

        add_graph("IMU Bias", "bg.x", latest_sliding_window_bg.x(), 0.1, -0.1);
        add_graph("IMU Bias", "bg.y", latest_sliding_window_bg.y(), 0.1, -0.1);
        add_graph("IMU Bias", "bg.z", latest_sliding_window_bg.z(), 0.1, -0.1);
        add_graph("IMU Bias", "ba.x", latest_sliding_window_ba.x(), 0.3, -0.3);
        add_graph("IMU Bias", "ba.y", latest_sliding_window_ba.y(), 0.3, -0.3);
        add_graph("IMU Bias", "ba.z", latest_sliding_window_ba.z(), 0.3, -0.3);

        // [5] visualization
        add_points(landmark_points, landmark_colors);
        add_path(sliding_window_trajectory, {1.0, 0.0, 1.0});
        add_path(output_trajectory, {0.0, 1.0, 1.0});

        shader.init(
            "simple_shader",
            glsl_code(
                uniform mat4 model_view_proj;
                uniform float scale;
                in vec3 point;
                in vec3 color;
                out vec3 c; void main() {
                    c = color;
                    gl_Position = model_view_proj * vec4(scale * point, 1.0);
                }),
            glsl_code(
                in vec3 c;
                out vec4 color;
                void main() {
                    color = vec4(c, 1.0);
                }));

        K = pvio_config->camera_intrinsic();

        output_writer = std::make_unique<TumOutputWriter>("trajectory.tum");
    }

    bool step() {
        static bool has_gyr = false, has_acc = false;
        while (true) {
            DatasetReader::NextDataType next_data_type;
            while ((next_data_type = dataset_reader->next()) == DatasetReader::AGAIN) {}
            switch (next_data_type) {
            case DatasetReader::AGAIN: { // impossible but we put it here
                break;
            }
            case DatasetReader::CAMERA: {
                auto image = dataset_reader->read_image();
#ifdef PVIO_ENABLE_THREADING
                // double t = time_now();
                // double dt = t - last_frame_time;
                // if (dt * TARGET_FPS <= 1.0) {
                //     double t_wait = std::max(1.0 / TARGET_FPS - dt, 0.001);
                //     std::this_thread::sleep_for(std::chrono::milliseconds((long long)(t_wait * 1000)));
                // }
                // last_frame_time = t;
                std::this_thread::sleep_for(std::chrono::milliseconds((long long)(20)));
#endif
                if (has_acc && has_gyr) {
                    pvio::OutputPose pose = pvio_instance->track_camera(image);
                    if (!pose.q.coeffs().isZero()) {
                        output_trajectory.emplace_back(pose.p);
                        output_writer->write_pose(image->t, pose);
                        latest_output_q = pose.q;
                        latest_output_p = pose.p;
                    }
                    visualize();
                }
                return true;
                break;
            }
            case DatasetReader::GYROSCOPE: {
                auto [t, gyr] = dataset_reader->read_gyroscope();
                pvio::OutputPose pose = pvio_instance->track_gyroscope(t, gyr.x(), gyr.y(), gyr.z());
                has_gyr = true;
                break;
            }
            case DatasetReader::ACCELEROMETER: {
                auto [t, acc] = dataset_reader->read_accelerometer();
                pvio::OutputPose pose = pvio_instance->track_accelerometer(t, acc.x(), acc.y(), acc.z());
                has_acc = true;
                break;
            }
            case DatasetReader::END: {
                return false;
            }
            }
        }
    }

    void shutdown() {
        exit(EXIT_SUCCESS);
    }

    void visualize() {
        notify(feature_tracker_image);
        notify(sliding_window_track_image);
        notify(sliding_window_reprojection_image);

        forensics(input_data_fps, fps) {
            if (fps.has_value()) {
                input_data_fps = std::any_cast<double>(fps);
                notify(input_data_fps);
            }
        }

        forensics(input_real_fps, fps) {
            if (fps.has_value()) {
                input_real_fps = std::any_cast<double>(fps);
                notify(input_real_fps);
            }
        }

        forensics(input_output_lag, lag) {
            if (lag.has_value()) {
                input_output_lag = std::any_cast<double>(lag) * 1000;
                notify(input_output_lag);
            }
        }

        forensics(feature_tracker_time, time) {
            if (time.has_value()) {
                feature_tracker_time = std::any_cast<double>(time) * 1000;
                notify(feature_tracker_time);
            }
        }

        forensics(bundle_adjustor_solve_time, time) {
            if (time.has_value()) {
                bundle_adjustor_solve_time = std::any_cast<double>(time) * 1000;
                notify(bundle_adjustor_solve_time);
            }
        }

        forensics(sliding_window_landmarks, landmarks) {
            if (landmarks.has_value()) {
                const auto &l = std::any_cast<std::vector<pvio::OutputMapPoint>>(landmarks);
                landmark_points.clear();
                landmark_colors.clear();
                for (auto &point : l) {
                    landmark_points.emplace_back(point.p);
                    if (point.reserved > 0) {
                        landmark_colors.emplace_back(plane_color(point.reserved - 1));
                    } else if (point.reserved < 0) {
                        landmark_colors.emplace_back(1.0, 0.0, 0.0);
                    } else {
                        landmark_colors.emplace_back(1.0, 1.0, 0.0);
                    }
                }
            }
        }

        forensics(sliding_window_planes, map_planes) {
            if (map_planes.has_value()) {
                landmark_planes = std::any_cast<std::vector<pvio::OutputPlane>>(map_planes);
            }
        }

        forensics(sliding_window_keyframe_poses, keyframe_poses) {
            if (keyframe_poses.has_value()) {
                const auto &p = std::any_cast<std::vector<pvio::OutputState>>(keyframe_poses);
                sliding_window_keyframe_qs.clear();
                sliding_window_keyframe_ps.clear();
                for (auto &pose : p) {
                    sliding_window_keyframe_ps.push_back(pose.p);
                    sliding_window_keyframe_qs.push_back(pose.q);
                }
                latest_sliding_window_q = p.back().q;
                latest_sliding_window_p = p.back().p;
                latest_sliding_window_bg = p.back().bg;
                latest_sliding_window_ba = p.back().ba;
                notify(latest_sliding_window_bg.x());
                notify(latest_sliding_window_bg.y());
                notify(latest_sliding_window_bg.z());
                notify(latest_sliding_window_ba.x());
                notify(latest_sliding_window_ba.y());
                notify(latest_sliding_window_ba.z());
                sliding_window_trajectory.push_back(p.back().p);
            }
        }
    }

  private:
    void draw() override {
        shader.bind();
        shader.setUniform("model_view_proj", model_view_proj());
        shader.setUniform("scale", world_scale());

        for (size_t i = 0; i < sliding_window_keyframe_qs.size(); ++i) {
            draw_camera(sliding_window_keyframe_ps[i], sliding_window_keyframe_qs[i], K, {0.0, 1.0, 1.0}, 0.0003);
        }
        draw_camera(latest_output_p, latest_output_q, K, {1.0, 1.0, 0.0}, 0.0005);
        for (const auto &plane : landmark_planes) {
            draw_plane(plane);
            std::vector<std::pair<pvio::vector<3>, pvio::vector<3>>> sector_lines;
            sector_lines.reserve(plane.vertices.size());
            for (const auto &vertex : plane.vertices) {
                sector_lines.emplace_back(plane.reference_point, vertex);
            }
            // draw_lines(sector_lines, plane_color(plane.id));
        }
    }

  private: // internal functions goes to the end
    pvio::matrix<3, 2> s2_tangential_basis_barrel(const pvio::vector<3> &x) {
        pvio::vector<3> b1 = x.cross(abs(x.z()) < 0.866 ? pvio::vector<3>::UnitZ() : pvio::vector<3>::UnitY()).normalized();
        pvio::vector<3> b2 = x.cross(b1).normalized();
        return (pvio::matrix<3, 2>() << b1, b2).finished();
    }

    Eigen::Vector3d plane_color(size_t id) {
        size_t h = id * 6364136223846793005u + 1442695040888963407;
        return {(h & 0xFF) / 512.0 + 0.5, ((h >> 4) & 0xFF) / 512.0 + 0.5, ((h >> 8) & 0xFF) / 512.0 + 0.5};
    }

    void draw_lines(const std::vector<std::pair<pvio::vector<3>, pvio::vector<3>>> &lines, const pvio::vector<3> &color = {0, 0.5, 0.5}) {
        Eigen::MatrixXf draw_point;
        Eigen::MatrixXf draw_color;
        draw_point.resize(3, lines.size() * 2);
        draw_color.resize(3, lines.size() * 2);
        for (size_t i = 0; i < lines.size(); ++i) {
            draw_point.col(i * 2) = lines[i].first.cast<float>();
            draw_point.col(i * 2 + 1) = lines[i].second.cast<float>();
            draw_color.col(i * 2) = color.cast<float>();
            draw_color.col(i * 2 + 1) = color.cast<float>();
        }
        shader.uploadAttrib("point", draw_point);
        shader.uploadAttrib("color", draw_color);
        shader.drawArray(GL_LINES, 0, draw_point.cols());
    }

    void draw_plane(const pvio::OutputPlane &plane) {
        Eigen::MatrixXf draw_point;
        Eigen::MatrixXf draw_color;
        draw_point.resize(3, 12);
        draw_color.resize(3, 12);
        pvio::matrix<3, 2> basis = s2_tangential_basis_barrel(plane.normal);
        std::array<pvio::vector<3>, 4> rect_points;
        rect_points[0] = plane.reference_point + 0.5 * basis.col(0) + 0.5 * basis.col(1);
        rect_points[1] = plane.reference_point - 0.5 * basis.col(0) + 0.5 * basis.col(1);
        rect_points[2] = plane.reference_point - 0.5 * basis.col(0) - 0.5 * basis.col(1);
        rect_points[3] = plane.reference_point + 0.5 * basis.col(0) - 0.5 * basis.col(1);
        draw_point.col(0) = rect_points[0].cast<float>();
        draw_point.col(1) = rect_points[1].cast<float>();
        draw_point.col(2) = rect_points[1].cast<float>();
        draw_point.col(3) = rect_points[2].cast<float>();
        draw_point.col(4) = rect_points[2].cast<float>();
        draw_point.col(5) = rect_points[3].cast<float>();
        draw_point.col(6) = rect_points[3].cast<float>();
        draw_point.col(7) = rect_points[0].cast<float>();
        draw_point.col(8) = rect_points[0].cast<float>();
        draw_point.col(9) = rect_points[2].cast<float>();
        draw_point.col(10) = rect_points[1].cast<float>();
        draw_point.col(11) = rect_points[3].cast<float>();
        Eigen::Vector3d color = plane_color(plane.id);
        for (int i = 0; i < 12; ++i) {
            draw_color.col(i) = color.cast<float>();
        }
        shader.uploadAttrib("point", draw_point);
        shader.uploadAttrib("color", draw_color);
        shader.drawArray(GL_LINES, 0, draw_point.cols());
    }
    void draw_camera(const pvio::vector<3> &p, const pvio::quaternion &q, const pvio::matrix<3> &K, const pvio::vector<3> &color, double size) {
        Eigen::MatrixXf draw_point;
        Eigen::MatrixXf draw_color;
        draw_point.resize(3, 16);
        draw_color.resize(3, 16);

        std::array<pvio::vector<3>, 5> cone_points;
        pvio::matrix<3> dcm = q.matrix();
        cone_points[0] = p;
        cone_points[1] = p + size * (dcm.col(2) * K(0, 0) + dcm.col(0) * K(0, 2) + dcm.col(1) * K(1, 2));
        cone_points[2] = p + size * (dcm.col(2) * K(0, 0) + dcm.col(0) * K(0, 2) - dcm.col(1) * K(1, 2));
        cone_points[3] = p + size * (dcm.col(2) * K(0, 0) - dcm.col(0) * K(0, 2) - dcm.col(1) * K(1, 2));
        cone_points[4] = p + size * (dcm.col(2) * K(0, 0) - dcm.col(0) * K(0, 2) + dcm.col(1) * K(1, 2));

        draw_point.col(0) = cone_points[0].cast<float>();
        draw_point.col(1) = cone_points[1].cast<float>();
        draw_point.col(2) = cone_points[0].cast<float>();
        draw_point.col(3) = cone_points[2].cast<float>();
        draw_point.col(4) = cone_points[0].cast<float>();
        draw_point.col(5) = cone_points[3].cast<float>();
        draw_point.col(6) = cone_points[0].cast<float>();
        draw_point.col(7) = cone_points[4].cast<float>();
        draw_point.col(8) = cone_points[1].cast<float>();
        draw_point.col(9) = cone_points[2].cast<float>();
        draw_point.col(10) = cone_points[2].cast<float>();
        draw_point.col(11) = cone_points[3].cast<float>();
        draw_point.col(12) = cone_points[3].cast<float>();
        draw_point.col(13) = cone_points[4].cast<float>();
        draw_point.col(14) = cone_points[4].cast<float>();
        draw_point.col(15) = cone_points[1].cast<float>();

        for (int i = 0; i < 16; ++i) {
            draw_color.col(i) = color.cast<float>();
        }

        shader.uploadAttrib("point", draw_point);
        shader.uploadAttrib("color", draw_color);
        shader.drawArray(GL_LINES, 0, draw_point.cols());
    }

    double last_frame_time;

    static double time_now() {
        return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }
};

int main(int argc, const char *argv[]) {
    if (argc < 3) {
        return EXIT_FAILURE;
    }
    PVIOPcVisualizer visualizer(argv[1], argv[2]);
    visualizer.show();
    nanovis::main(1);
    return 0;
}
