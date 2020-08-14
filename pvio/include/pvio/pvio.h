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
#ifndef PVIO_PVIO_H
#define PVIO_PVIO_H

#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <pvio/version.h>

namespace pvio {

template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false, typename T = double>
using matrix = typename std::conditional<
    Rows != 1 && Cols != 1,
    Eigen::Matrix<T, Rows, Cols, UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
    Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false, typename T = double>
using vector = typename std::conditional<
    RowVector,
    matrix<1, Dimension, false, T>,
    matrix<Dimension, 1, false, T>>::type;

using quaternion = Eigen::Quaternion<double>;

struct OutputPose {
    quaternion q;
    vector<3> p;
};

struct OutputState {
    double t;
    quaternion q;
    vector<3> p;
    vector<3> v;
    vector<3> bg;
    vector<3> ba;
};

struct OutputMapPoint {
    vector<3> p;
    int reserved;
};

struct OutputPlane {
    vector<3> normal;
    double distance;
    vector<3> reference_point;
    std::vector<size_t> track_ids;
    size_t id;
    std::vector<vector<3>> vertices;
};

class Config {
  public:
    virtual ~Config();

    virtual matrix<3> camera_intrinsic() const = 0;
    virtual quaternion camera_to_body_rotation() const = 0;
    virtual vector<3> camera_to_body_translation() const = 0;
    virtual quaternion imu_to_body_rotation() const = 0;
    virtual vector<3> imu_to_body_translation() const = 0;

    virtual matrix<2> keypoint_noise_cov() const = 0;
    virtual matrix<3> gyroscope_noise_cov() const = 0;
    virtual matrix<3> accelerometer_noise_cov() const = 0;
    virtual matrix<3> gyroscope_bias_noise_cov() const = 0;
    virtual matrix<3> accelerometer_bias_noise_cov() const = 0;

    virtual double plane_distance_cov() const;

    virtual quaternion output_to_body_rotation() const;
    virtual vector<3> output_to_body_translation() const;

    virtual size_t sliding_window_size() const;

    virtual double feature_tracker_min_keypoint_distance() const;
    virtual size_t feature_tracker_max_keypoint_detection() const;
    virtual size_t feature_tracker_max_init_frames() const;
    virtual size_t feature_tracker_max_frames() const;
    virtual bool feature_tracker_predict_keypoints() const;

    virtual size_t initializer_keyframe_gap() const;
    virtual size_t initializer_min_matches() const;
    virtual double initializer_min_parallax() const;
    virtual size_t initializer_min_triangulation() const;
    virtual size_t initializer_min_landmarks() const;
    virtual bool initializer_refine_imu() const;

    virtual size_t solver_iteration_limit() const;
    virtual double solver_time_limit() const;

    virtual int random() const;

    void log_config() const;
};

class Image {
  public:
    double t;

    virtual size_t width() const = 0;
    virtual size_t height() const = 0;

    virtual size_t level_num() const {
        return 0;
    }

    virtual double evaluate(const vector<2> &u, int level = 0) const = 0;
    virtual double evaluate(const vector<2> &u, vector<2> &ddu, int level = 0) const = 0;

    virtual ~Image() = default;
    virtual void preprocess() {
    }
    virtual void detect_keypoints(std::vector<vector<2>> &keypoints, size_t max_points = 0, double keypoint_distance = 0.5) const = 0;
    virtual void track_keypoints(const Image *next_image, const std::vector<vector<2>> &curr_keypoints, std::vector<vector<2>> &next_keypoints, std::vector<char> &result_status) const = 0;
};

class PVIO {
    class Core;

  public:
    PVIO(std::shared_ptr<Config> config);
    ~PVIO();

    OutputPose track_gyroscope(const double &t, const double &x, const double &y, const double &z);
    OutputPose track_accelerometer(const double &t, const double &x, const double &y, const double &z);
    OutputPose track_camera(std::shared_ptr<Image> image);

  private:
    std::unique_ptr<Core> core;
};

} // namespace pvio

#endif // PVIO_PVIO_H
