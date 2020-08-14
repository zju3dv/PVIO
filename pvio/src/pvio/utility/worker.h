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
#ifndef PVIO_WORKER_H
#define PVIO_WORKER_H

#include <pvio/common.h>

namespace pvio {

class Worker {
  public:
    Worker() {
    }

    virtual ~Worker() {
        stop();
    }

    void start() {
        worker_running = true;
#if defined(PVIO_ENABLE_THREADING)
        worker_thread = std::thread(&Worker::worker_loop, this);
#endif
    }

    void stop() {
        if (worker_running) {
            worker_running = false;
#if defined(PVIO_ENABLE_THREADING)
            worker_cv.notify_all();
            worker_thread.join();
#endif
        }
    }

    std::unique_lock<std::mutex> lock() const {
        return std::unique_lock(worker_mutex);
    }

    void resume(std::unique_lock<std::mutex> &l) {
        l.unlock();
#if defined(PVIO_ENABLE_THREADING)
        worker_cv.notify_all();
#else
        worker_loop();
#endif
    }

    virtual bool empty() const = 0;
    virtual void work(std::unique_lock<std::mutex> &l) = 0;

  protected:
    std::atomic<bool> worker_running;

  private:
    void worker_loop();

#if defined(PVIO_ENABLE_THREADING)
    std::thread worker_thread;
    std::condition_variable worker_cv;
#endif
    mutable std::mutex worker_mutex;
};

} // namespace pvio

#endif // PVIO_WORKER_H
