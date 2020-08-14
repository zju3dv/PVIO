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
#ifndef PVIO_FEATURE_TRACKER_H
#define PVIO_FEATURE_TRACKER_H

#include <pvio/core/core.h>
#include <pvio/estimation/state.h>
#include <pvio/utility/worker.h>

namespace pvio {

class PVIO::Core::FeatureTracker : public Worker {
  public:
    FeatureTracker(PVIO::Core *core, std::shared_ptr<Config> config);
    ~FeatureTracker();

    bool empty() const override {
        return frames.empty();
    }

    void work(std::unique_lock<std::mutex> &l) override;

    void track_frame(std::unique_ptr<Frame> frame);

    std::optional<std::tuple<double, PoseState, MotionState>> get_latest_state() const;

    std::unique_ptr<Map> map;

  private:
    PVIO::Core *core;
    std::deque<std::unique_ptr<Frame>> frames;
    std::shared_ptr<Config> config;
    std::optional<std::tuple<double, PoseState, MotionState>> latest_state;
    mutable std::mutex latest_pose_mutex;
};

} // namespace pvio

#endif // PVIO_FEATURE_TRACKER_H
