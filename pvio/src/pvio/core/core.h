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
#ifndef PVIO_CORE_H
#define PVIO_CORE_H

#include <pvio/common.h>
#include <pvio/pvio.h>

namespace pvio {

class Config;
class Frame;
class Image;
class Map;

class PVIO::Core {
    struct GyroscopeData {
        double t;
        vector<3> w;
    };
    struct AccelerometerData {
        double t;
        vector<3> a;
    };

  public:
    class FeatureTracker;
    class FrontendWorker;

    Core(std::shared_ptr<Config> config);
    virtual ~Core();

    const Config *configurations() const;

    OutputPose track_gyroscope(const double &t, const double &x, const double &y, const double &z);
    OutputPose track_accelerometer(const double &t, const double &x, const double &y, const double &z);
    OutputPose track_camera(std::shared_ptr<Image> image);

    std::unique_ptr<FeatureTracker> feature_tracker;
    std::unique_ptr<FrontendWorker> frontend;

  private:
    void track_imu(const ImuData &imu);
    OutputPose predict_pose(const double &t);

    void measure_camera_input_rate(const double &t) const;

    std::deque<GyroscopeData> gyroscopes;
    std::deque<AccelerometerData> accelerometers;

    std::deque<ImuData> imus;
    std::deque<std::unique_ptr<Frame>> frames;
    std::deque<ImuData> frontal_imus;

    std::shared_ptr<Config> config;
};

} // namespace pvio

#endif // PVIO_CORE_H
