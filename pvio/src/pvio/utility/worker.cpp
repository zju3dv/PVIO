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
#include <pvio/utility/worker.h>

namespace pvio {

void Worker::worker_loop() {
    while (worker_running) {
        auto l = lock();
#if defined(PVIO_ENABLE_THREADING)
        if (worker_running && empty()) {
            worker_cv.wait(l, [this] { return !worker_running || !empty(); });
        }
#else
        if (empty()) break;
#endif
        if (!worker_running) break;
        work(l);
    }
}

} // namespace pvio
