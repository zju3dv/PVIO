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
#ifndef PVIO_SLIDING_WINDOW_TRACKER_H
#define PVIO_SLIDING_WINDOW_TRACKER_H

#include <pvio/core/core.h>
#include <pvio/estimation/state.h>

namespace pvio {

class Config;
class Frame;
class Map;
class PlaneExtractor;

class SlidingWindowTracker {
  public:
    SlidingWindowTracker(std::unique_ptr<Map> keyframe_map, std::shared_ptr<Config> config);
    ~SlidingWindowTracker();

    void mirror_frame(Map *feature_tracking_map, size_t frame_id);

    bool track();

    std::tuple<PoseState, MotionState> get_latest_state() const;

    std::unique_ptr<Map> map;

  private:
    void keyframe_check(Frame *frame);

    std::unique_ptr<Frame> frame;
    std::unique_ptr<PlaneExtractor> plane_extractor;
    std::shared_ptr<Config> config;
    size_t skipped_frames;
};

} // namespace pvio

#endif // PVIO_SLIDING_WINDOW_TRACKER_H
