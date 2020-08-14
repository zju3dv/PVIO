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
#ifndef PVIO_CERES_SOLVER_OPTIONS_H
#define PVIO_CERES_SOLVER_OPTIONS_H

#include <ceres/ceres.h>
#include <pvio/common.h>

namespace pvio {

inline void set_solver_options(ceres::Solver::Options &solver_options, size_t max_iter, double max_time) {
    solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
    solver_options.trust_region_strategy_type = ceres::DOGLEG;
    solver_options.max_num_iterations = (int)max_iter;
    solver_options.max_solver_time_in_seconds = max_time;
    solver_options.num_threads = 1;
    solver_options.update_state_every_iteration = true;
}

} // namespace pvio

#endif // PVIO_CERES_SOLVER_OPTIONS_H
