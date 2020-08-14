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
#include <pvio/common.h>
#include <pvio/core/core.h>
#include <pvio/core/feature_tracker.h>
#include <pvio/core/frontend_worker.h>
#include <pvio/forensics.h>
#include <pvio/geometry/lie_algebra.h>
#include <pvio/geometry/stereo.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>

namespace pvio {

static const OutputPose invalid_output_pose = {{0, 0, 0, 0}, {0, 0, 0}};

static void propagate_state(double &state_time, PoseState &state_pose, MotionState &state_motion, double t, const vector<3> &w, const vector<3> &a) {
    static const vector<3> gravity = {0, 0, -PVIO_GRAVITY_NOMINAL};
    double dt = t - state_time;
    state_pose.p = state_pose.p + dt * state_motion.v + 0.5 * dt * dt * (gravity + state_pose.q * (a - state_motion.ba));
    state_motion.v = state_motion.v + dt * (gravity + state_pose.q * (a - state_motion.ba));
    state_pose.q = (state_pose.q * expmap((w - state_motion.bg) * dt)).normalized();
    state_time = t;
}

PVIO::Core::Core(std::shared_ptr<Config> config) :
    config(config) {
    config->log_config();
    frontend = std::make_unique<FrontendWorker>(this, config);
    frontend->start();
    feature_tracker = std::make_unique<FeatureTracker>(this, config);
    feature_tracker->start();
}

PVIO::Core::~Core() {
    feature_tracker->stop();
    frontend->stop();
}

const Config *PVIO::Core::configurations() const {
    return config.get();
}

OutputPose PVIO::Core::track_gyroscope(const double &t, const double &x, const double &y, const double &z) {
    if (accelerometers.size() > 0) {
        if (t < accelerometers.front().t) {
            gyroscopes.clear();
        } else {
            while (accelerometers.size() > 0 && t >= accelerometers.front().t) {
                const auto &acc = accelerometers.front();
                double lambda = (acc.t - gyroscopes[0].t) / (t - gyroscopes[0].t);
                vector<3> w = gyroscopes[0].w + lambda * (vector<3>{x, y, z} - gyroscopes[0].w);
                track_imu({acc.t, w, acc.a});
                accelerometers.pop_front();
            }
            if (accelerometers.size() > 0) {
                while (gyroscopes.size() > 0 && gyroscopes.front().t < t) {
                    gyroscopes.pop_front();
                }
            }
        }
    }
    gyroscopes.emplace_back(GyroscopeData{t, {x, y, z}});
    return predict_pose(t);
}

OutputPose PVIO::Core::track_accelerometer(const double &t, const double &x, const double &y, const double &z) {
    if (gyroscopes.size() > 0 && t >= gyroscopes.front().t) {
        if (t > gyroscopes.back().t) {
            while (gyroscopes.size() > 1) {
                gyroscopes.pop_front();
            }
            // t > gyroscopes[0].t
            accelerometers.emplace_back(AccelerometerData{t, {x, y, z}});
        } else if (t == gyroscopes.back().t) {
            while (gyroscopes.size() > 1) {
                gyroscopes.pop_front();
            }
            track_imu({t, gyroscopes.front().w, {x, y, z}});
        } else {
            // pre-condition: gyroscopes.front().t <= t < gyroscopes.back().t  ==>  gyroscopes.size() >= 2
            while (t >= gyroscopes[1].t) {
                gyroscopes.pop_front();
            }
            // post-condition: t < gyroscopes[1].t
            double lambda = (t - gyroscopes[0].t) / (gyroscopes[1].t - gyroscopes[0].t);
            vector<3> w = gyroscopes[0].w + lambda * (gyroscopes[1].w - gyroscopes[0].w);
            track_imu({t, w, {x, y, z}});
        }
    }
    return predict_pose(t);
}

OutputPose PVIO::Core::track_camera(std::shared_ptr<Image> image) {
    measure_camera_input_rate(image->t);
    std::unique_ptr<Frame> frame = std::make_unique<Frame>();
    frame->K = config->camera_intrinsic();
    frame->image = image;
    frame->sqrt_inv_cov = frame->K.block<2, 2>(0, 0);
    frame->sqrt_inv_cov(0, 0) /= ::sqrt(config->keypoint_noise_cov()(0, 0));
    frame->sqrt_inv_cov(1, 1) /= ::sqrt(config->keypoint_noise_cov()(1, 1));
    frame->camera.q_cs = config->camera_to_body_rotation();
    frame->camera.p_cs = config->camera_to_body_translation();
    frame->imu.q_cs = config->imu_to_body_rotation();
    frame->imu.p_cs = config->imu_to_body_translation();
    frame->preintegration.cov_a = config->accelerometer_noise_cov();
    frame->preintegration.cov_w = config->gyroscope_noise_cov();
    frame->preintegration.cov_ba = config->accelerometer_bias_noise_cov();
    frame->preintegration.cov_bg = config->gyroscope_bias_noise_cov();
    frames.emplace_back(std::move(frame));
    return predict_pose(image->t);
}

void PVIO::Core::track_imu(const ImuData &imu) {
    frontal_imus.emplace_back(imu);
    imus.emplace_back(imu);
    while (imus.size() > 0 && frames.size() > 0) {
        if (imus.front().t <= frames.front()->image->t) {
            frames.front()->preintegration.data.push_back(imus.front());
            imus.pop_front();
        } else {
            feature_tracker->track_frame(std::move(frames.front()));
            frames.pop_front();
        }
    }
}

OutputPose PVIO::Core::predict_pose(const double &t) {
    if (auto maybe_state = feature_tracker->get_latest_state()) {
        auto [state_time, state_pose, state_motion] = maybe_state.value();
        critical_forensics(input_output_lag, lag) {
            lag = std::min(t - state_time, 5.0);
        }
        while (!frontal_imus.empty() && frontal_imus.front().t <= state_time) {
            frontal_imus.pop_front();
        }
        for (const auto &imu : frontal_imus) {
            if (imu.t <= t) {
                propagate_state(state_time, state_pose, state_motion, imu.t, imu.w, imu.a);
            }
        }
        OutputPose output_pose;
        output_pose.q = state_pose.q * config->output_to_body_rotation();
        output_pose.p = state_pose.p + state_pose.q * config->output_to_body_translation();
        return output_pose;
    } else {
        return invalid_output_pose;
    }
}

void PVIO::Core::measure_camera_input_rate(const double &t) const {
    static const int fw = 10;
    static double old_data_t = std::numeric_limits<double>::quiet_NaN();
    static double old_real_t = std::numeric_limits<double>::quiet_NaN();
    static double data_dt_stat = 1.0;
    static double real_dt_stat = 1.0;

    double real_t = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    if (std::isnormal(old_data_t)) {
        double data_dt = std::max(std::abs(t - old_data_t), 1.0e-7);
        double real_dt = std::max(std::abs(real_t - old_real_t), 1.0e-7);
        data_dt_stat = (data_dt_stat * (fw - 1) + data_dt) / fw;
        real_dt_stat = (real_dt_stat * (fw - 1) + real_dt) / fw;
        critical_forensics(input_data_fps, fps) {
            fps = 1.0 / data_dt_stat;
        }
        critical_forensics(input_real_fps, fps) {
            fps = 1.0 / real_dt_stat;
        }
    }
    old_data_t = t;
    old_real_t = real_t;
}

} // namespace pvio
