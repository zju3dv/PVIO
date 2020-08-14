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
#ifndef PVIO_EXTRA_YAML_CONFIG_H
#define PVIO_EXTRA_YAML_CONFIG_H

#include <stdexcept>
#include <pvio/pvio.h>

namespace pvio::extra {

class YamlConfig : public Config {
  public:
    struct Exception : public std::runtime_error {
        Exception(const std::string &what) :
            std::runtime_error(what) {
        }
    };
    struct LoadException : public Exception {
        LoadException(const std::string &filename) :
            Exception("cannot load config " + filename) {
        }
    };
    struct ParseException : public Exception {
        ParseException(const std::string &message) :
            Exception(message) {
        }
    };
    struct ConfigMissingException : public Exception {
        ConfigMissingException(const std::string &config_path) :
            Exception("config \"" + config_path + "\" is mandatory") {
        }
    };
    struct TypeErrorException : public Exception {
        TypeErrorException(const std::string &config_path) :
            Exception("config \"" + config_path + "\" has wrong type") {
        }
    };

    YamlConfig(const std::string &filename);
    ~YamlConfig();

    matrix<3> camera_intrinsic() const override;
    quaternion camera_to_body_rotation() const override;
    vector<3> camera_to_body_translation() const override;
    quaternion imu_to_body_rotation() const override;
    vector<3> imu_to_body_translation() const override;

    matrix<2> keypoint_noise_cov() const override;
    matrix<3> gyroscope_noise_cov() const override;
    matrix<3> accelerometer_noise_cov() const override;
    matrix<3> gyroscope_bias_noise_cov() const override;
    matrix<3> accelerometer_bias_noise_cov() const override;

    double plane_distance_cov() const override;

    quaternion output_to_body_rotation() const override;
    vector<3> output_to_body_translation() const override;

    size_t sliding_window_size() const override;

    double feature_tracker_min_keypoint_distance() const override;
    size_t feature_tracker_max_keypoint_detection() const override;
    size_t feature_tracker_max_init_frames() const override;
    size_t feature_tracker_max_frames() const override;
    bool feature_tracker_predict_keypoints() const override;

    size_t initializer_keyframe_gap() const override;
    size_t initializer_min_matches() const override;
    double initializer_min_parallax() const override;
    size_t initializer_min_triangulation() const override;
    size_t initializer_min_landmarks() const override;
    bool initializer_refine_imu() const override;

    size_t solver_iteration_limit() const override;
    double solver_time_limit() const override;

  private:
    matrix<3> m_camera_intrinsic;
    quaternion m_camera_to_body_rotation;
    vector<3> m_camera_to_body_translation;
    quaternion m_imu_to_body_rotation;
    vector<3> m_imu_to_body_translation;
    matrix<2> m_keypoint_noise_cov;
    matrix<3> m_gyroscope_noise_cov;
    matrix<3> m_accelerometer_noise_cov;
    matrix<3> m_gyroscope_bias_noise_cov;
    matrix<3> m_accelerometer_bias_noise_cov;

    double m_plane_distance_cov;

    quaternion m_output_to_body_rotation;
    vector<3> m_output_to_body_translation;

    size_t m_sliding_window_size;

    double m_feature_tracker_min_keypoint_distance;
    size_t m_feature_tracker_max_keypoint_detection;
    size_t m_feature_tracker_max_init_frames;
    size_t m_feature_tracker_max_frames;
    bool m_feature_tracker_predict_keypoints;

    size_t m_initializer_keyframe_gap;
    size_t m_initializer_min_matches;
    double m_initializer_min_parallax;
    size_t m_initializer_min_triangulation;
    size_t m_initializer_min_landmarks;
    bool m_initializer_refine_imu;

    size_t m_solver_iteration_limit;
    double m_solver_time_limit;
};

} // namespace pvio::extra

#endif // PVIO_EXTRA_YAML_CONFIG_H
