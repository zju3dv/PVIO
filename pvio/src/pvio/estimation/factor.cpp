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
#include <pvio/estimation/ceres/marginalization_error_cost.h>
#include <pvio/estimation/ceres/preintegration_error_cost.h>
#include <pvio/estimation/ceres/reprojection_error_cost.h>
#include <pvio/estimation/factor.h>
#include <pvio/map/frame.h>
#include <pvio/map/track.h>

namespace pvio {

std::unique_ptr<Factor> Factor::create_marginalization_error(const matrix<> &sqrt_inv_cov, const vector<> &infovec, std::vector<Frame *> &&frames) {
    return std::make_unique<Factor>(
        std::make_unique<MarginalizationErrorCost>(sqrt_inv_cov, infovec, std::move(frames)),
        factor_construct_t());
}

std::unique_ptr<Factor> Factor::create_reprojection_error(Track *track, Frame *frame, size_t keypoint_index) {
    return std::make_unique<Factor>(
        std::make_unique<ReprojectionErrorCost>(track, frame, keypoint_index),
        factor_construct_t());
}

std::unique_ptr<Factor> Factor::create_preintegration_error(Frame *frame_i, Frame *frame_j) {
    return std::make_unique<Factor>(
        std::make_unique<PreIntegrationErrorCost>(frame_i, frame_j),
        factor_construct_t());
}

Factor::Factor(std::unique_ptr<FactorCostFunction> cost_function, const factor_construct_t &) :
    cost_function(std::move(cost_function)) {
}

Factor::~Factor() = default;

} // namespace pvio
