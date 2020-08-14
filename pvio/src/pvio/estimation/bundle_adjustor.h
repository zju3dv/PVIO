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
#ifndef PVIO_BUNDLE_ADJUSTOR_H
#define PVIO_BUNDLE_ADJUSTOR_H

#include <pvio/common.h>
#include <pvio/estimation/state.h>

namespace pvio {

class Map;
class Frame;

class BundleAdjustor {
    struct BundleAdjustorSolver; // pimpl

  public:
    BundleAdjustor();
    virtual ~BundleAdjustor();

    bool solve(Map *map, Config *config, bool use_inertial = true);
    double compute_reprojection_error(Map *map);
    void marginalize_frame(Map *map, size_t index);

  private:
    std::unique_ptr<BundleAdjustorSolver> solver;
};

} // namespace pvio

#endif // PVIO_BUNDLE_ADJUSTOR_H
