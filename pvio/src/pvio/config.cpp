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

namespace pvio {

Config::~Config() = default;

double Config::plane_distance_cov() const {
    return 0.01 * 0.01;
}

quaternion Config::output_to_body_rotation() const {
    return quaternion::Identity();
}
vector<3> Config::output_to_body_translation() const {
    return vector<3>::Zero();
}

size_t Config::sliding_window_size() const {
    return 10;
}

double Config::feature_tracker_min_keypoint_distance() const {
    return 20.0;
}

size_t Config::feature_tracker_max_keypoint_detection() const {
    return 150;
}

size_t Config::feature_tracker_max_init_frames() const {
    return 60;
}

size_t Config::feature_tracker_max_frames() const {
    return 20;
}

bool Config::feature_tracker_predict_keypoints() const {
    return true;
}

size_t Config::initializer_keyframe_gap() const {
    return 5;
}

size_t Config::initializer_min_matches() const {
    return 50;
}

double Config::initializer_min_parallax() const {
    return 10;
}

size_t Config::initializer_min_triangulation() const {
    return 20;
}

size_t Config::initializer_min_landmarks() const {
    return 30;
}

bool Config::initializer_refine_imu() const {
    return true;
}

size_t Config::solver_iteration_limit() const {
    return 10;
}

double Config::solver_time_limit() const {
    return 1.0e6;
}

int Config::random() const {
    return 648;
}

void Config::log_config() const {
    std::stringstream ss;
    ss << std::scientific
       << std::boolalpha
       << std::setprecision(5);

    ss << "Config::camera_intrinsic:\n"
       << camera_intrinsic() << "\n"
       << std::endl;

    ss << "Config::camera_to_body_rotation:\n"
       << camera_to_body_rotation().coeffs().transpose() << "\n"
       << std::endl;

    ss << "Config::camera_to_body_translation:\n"
       << camera_to_body_translation().transpose() << "\n"
       << std::endl;

    ss << "Config::imu_to_body_rotation:\n"
       << imu_to_body_rotation().coeffs().transpose() << "\n"
       << std::endl;

    ss << "Config::imu_to_body_translation:\n"
       << imu_to_body_translation().transpose() << "\n"
       << std::endl;

    ss << "Config::keypoint_noise_cov:\n"
       << keypoint_noise_cov() << "\n"
       << std::endl;

    ss << "Config::gyroscope_noise_cov:\n"
       << gyroscope_noise_cov() << "\n"
       << std::endl;

    ss << "Config::accelerometer_noise_cov:\n"
       << accelerometer_noise_cov() << "\n"
       << std::endl;

    ss << "Config::gyroscope_bias_noise_cov:\n"
       << gyroscope_bias_noise_cov() << "\n"
       << std::endl;

    ss << "Config::accelerometer_bias_noise_cov:\n"
       << accelerometer_bias_noise_cov() << "\n"
       << std::endl;

    ss << "Config::plane_distance_cov:\n"
       << plane_distance_cov() << "\n"
       << std::endl;

    ss << "Config::sliding_window_size: "
       << sliding_window_size()
       << std::endl;

    ss << "Config::feature_tracker_min_keypoint_distance: "
       << feature_tracker_min_keypoint_distance()
       << std::endl;

    ss << "Config::feature_tracker_max_keypoint_detection: "
       << feature_tracker_max_keypoint_detection()
       << std::endl;

    ss << "Config::feature_tracker_max_frames: "
       << feature_tracker_max_frames()
       << std::endl;

    ss << "Config::feature_tracker_predict_keypoints: "
       << feature_tracker_predict_keypoints()
       << std::endl;

    ss << "Config::initializer_keyframe_gap: "
       << initializer_keyframe_gap()
       << std::endl;

    ss << "Config::initializer_min_matches: "
       << initializer_min_matches()
       << std::endl;

    ss << "Config::initializer_min_parallax: "
       << initializer_min_parallax()
       << std::endl;

    ss << "Config::initializer_min_triangulation: "
       << initializer_min_triangulation()
       << std::endl;

    ss << "Config::initializer_min_landmarks: "
       << initializer_min_landmarks()
       << std::endl;

    ss << "Config::initializer_refine_imu: "
       << initializer_refine_imu()
       << std::endl;

    ss << "Config::solver_iteration_limit: "
       << solver_iteration_limit()
       << std::endl;

    ss << "Config::solver_time_limit: "
       << solver_time_limit()
       << std::endl;

    log_message(PVIO_LOG_INFO, "Configurations: \n%s", ss.str().c_str());
}

} // namespace pvio
