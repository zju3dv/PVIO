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
#ifndef PVIO_INITIALIZER_H
#define PVIO_INITIALIZER_H

#include <pvio/core/core.h>

namespace pvio {

class Config;
class Map;
class SlidingWindowTracker;

class Initializer {
  public:
    Initializer(std::shared_ptr<Config> config);
    ~Initializer();

    void mirror_keyframe_map(Map *feature_tracking_map, size_t init_frame_id);
    std::unique_ptr<SlidingWindowTracker> initialize();

    std::unique_ptr<Map> map;

  private:
    bool init_sfm();
    bool init_imu();

    void solve_gyro_bias();
    void solve_gravity_scale_velocity();
    void refine_scale_velocity_via_gravity();

    void reset_states();
    void preintegrate();
    bool apply_init(bool apply_ba = false, bool apply_velocity = true);

    vector<3> bg;
    vector<3> ba;
    vector<3> gravity;
    double scale;
    std::vector<vector<3>> velocities;

    std::shared_ptr<Config> config;
};

} // namespace pvio

#endif // PVIO_INITIALIZER_H
