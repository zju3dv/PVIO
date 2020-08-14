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
#ifndef PVIO_FRONTEND_WORKER_H
#define PVIO_FRONTEND_WORKER_H

#include <pvio/common.h>
#include <pvio/core/core.h>
#include <pvio/estimation/state.h>
#include <pvio/utility/worker.h>

namespace pvio {

class Config;
class Frame;
class Initializer;
class SlidingWindowTracker;

class PVIO::Core::FrontendWorker : public Worker {
  public:
    FrontendWorker(PVIO::Core *core, std::shared_ptr<Config> config);
    ~FrontendWorker();

    bool empty() const override;
    void work(std::unique_lock<std::mutex> &l) override;

    void issue_frame(Frame *frame);

    std::tuple<size_t, PoseState, MotionState> get_latest_state() const;

  private:
    std::deque<size_t> pending_frame_ids;

    PVIO::Core *core;
    std::shared_ptr<Config> config;
    std::unique_ptr<Initializer> initializer;
    std::unique_ptr<SlidingWindowTracker> sliding_window_tracker;

    std::tuple<size_t, PoseState, MotionState> latest_state;
    mutable std::mutex latest_state_mutex;
};

} // namespace pvio

#endif // PVIO_FRONTEND_WORKER_H
