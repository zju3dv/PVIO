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
#include <pvio/core/initializer.h>
#include <pvio/core/sliding_window_tracker.h>
#include <pvio/geometry/stereo.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/track.h>

namespace pvio {

PVIO::Core::FrontendWorker::FrontendWorker(PVIO::Core *core, std::shared_ptr<Config> config) :
    core(core), config(config) {
    initializer = std::make_unique<Initializer>(config);
    latest_state = {nil(), {}, {}};
}

PVIO::Core::FrontendWorker::~FrontendWorker() = default;

bool PVIO::Core::FrontendWorker::empty() const {
    return pending_frame_ids.empty();
}

void PVIO::Core::FrontendWorker::work(std::unique_lock<std::mutex> &l) {
    if (initializer) {
        size_t pending_frame_id = pending_frame_ids.front();
        pending_frame_ids.clear();
        l.unlock();
        synchronized(core->feature_tracker->map) {
            initializer->mirror_keyframe_map(core->feature_tracker->map.get(), pending_frame_id);
        }
        if ((sliding_window_tracker = initializer->initialize())) {
            std::unique_lock lk(latest_state_mutex);
            auto [pose, motion] = sliding_window_tracker->get_latest_state();
            latest_state = {pending_frame_id, pose, motion};
            lk.unlock();
            initializer.reset();
        }
    } else if (sliding_window_tracker) {
        size_t pending_frame_id = pending_frame_ids.front();
        pending_frame_ids.pop_front();
        // log_message(PVIO_LOG_DEBUG, "tracker::frames remaining = %zd", pending_frame_ids.size());
        l.unlock();
        synchronized(core->feature_tracker->map) {
            sliding_window_tracker->mirror_frame(core->feature_tracker->map.get(), pending_frame_id);
        }
        if (sliding_window_tracker->track()) {
            std::unique_lock lk(latest_state_mutex);
            auto [pose, motion] = sliding_window_tracker->get_latest_state();
            latest_state = {pending_frame_id, pose, motion};
            lk.unlock();
        } else {
            std::unique_lock lk(latest_state_mutex);
            latest_state = {nil(), {}, {}};
            lk.unlock();
            initializer = std::make_unique<Initializer>(config);
            sliding_window_tracker.reset();
        }
    }
}

void PVIO::Core::FrontendWorker::issue_frame(Frame *frame) {
    auto l = lock();
    pending_frame_ids.push_back(frame->id());
    resume(l);
}

std::tuple<size_t, PoseState, MotionState> PVIO::Core::FrontendWorker::get_latest_state() const {
    std::unique_lock lk(latest_state_mutex);
    return latest_state;
}

} // namespace pvio
