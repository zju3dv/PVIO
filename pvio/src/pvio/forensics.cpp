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
#include <pvio/forensics.h>

namespace pvio {

struct ForensicsSupport::VersionTag {
    int major;
    int minor;
    int patch;
};

ForensicsSupport::ForensicsSupport(const VersionTag &tag) :
    storage(ITEM_COUNT) {
    storage[RESERVED].first = tag;
}

ForensicsSupport::~ForensicsSupport() = default;

std::pair<std::any &, std::unique_lock<std::mutex>> ForensicsSupport::get(ForensicsItem item) {
    auto &si = support().storage[item];
    return {si.first, std::unique_lock(si.second)};
}

ForensicsSupport &ForensicsSupport::support() {
    static std::unique_ptr<ForensicsSupport> s_support = std::make_unique<ForensicsSupport>(VersionTag{PVIO_MAJOR_VERSION, PVIO_MINOR_VERSION, PVIO_PATCH_VERSION});
    return *s_support;
}

} // namespace pvio
