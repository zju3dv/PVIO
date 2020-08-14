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

namespace pvio {

PVIO::PVIO(std::shared_ptr<Config> config) {
    core = std::make_unique<PVIO::Core>(config);
}

PVIO::~PVIO() = default;

OutputPose PVIO::track_gyroscope(const double &t, const double &x, const double &y, const double &z) {
    return core->track_gyroscope(t, x, y, z);
}

OutputPose PVIO::track_accelerometer(const double &t, const double &x, const double &y, const double &z) {
    return core->track_accelerometer(t, x, y, z);
}

OutputPose PVIO::track_camera(std::shared_ptr<Image> image) {
    return core->track_camera(image);
}

} // namespace pvio
