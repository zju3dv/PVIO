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
#ifndef PVIO_MARGINALIZATION_ERROR_COST_H
#define PVIO_MARGINALIZATION_ERROR_COST_H

#include <ceres/ceres.h>
#include <pvio/common.h>
#include <pvio/estimation/factor.h>
#include <pvio/estimation/state.h>
#include <pvio/geometry/lie_algebra.h>
#include <pvio/map/frame.h>

namespace pvio {

class MarginalizationErrorCost : public Factor::FactorCostFunction, public ceres::CostFunction {
  public:
    MarginalizationErrorCost(const matrix<> &sqrt_inv_cov, const vector<> &infovec, std::vector<Frame *> &&frames) :
        sqrt_inv_cov(sqrt_inv_cov), infovec(infovec), frames(std::move(frames)) {
        set_num_residuals((int)this->frames.size() * ES_SIZE);
        pose_0.resize(this->frames.size());
        motion_0.resize(this->frames.size());

        mutable_parameter_block_sizes()->clear();
        for (size_t i = 0; i < this->frames.size(); ++i) {
            mutable_parameter_block_sizes()->push_back(4); // q
            mutable_parameter_block_sizes()->push_back(3); // p
            mutable_parameter_block_sizes()->push_back(3); // v
            mutable_parameter_block_sizes()->push_back(3); // bg
            mutable_parameter_block_sizes()->push_back(3); // ba
            pose_0[i] = this->frames[i]->pose;
            motion_0[i] = this->frames[i]->motion;
        }
    }

    void update() override {
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        for (size_t i = 0; i < frames.size(); ++i) {
            const_map<quaternion> q(parameters[5 * i + 0]);
            const_map<vector<3>> p(parameters[5 * i + 1]);
            const_map<vector<3>> v(parameters[5 * i + 2]);
            const_map<vector<3>> bg(parameters[5 * i + 3]);
            const_map<vector<3>> ba(parameters[5 * i + 4]);
            map<vector<3>> rq(&residuals[ES_SIZE * i + ES_Q]);
            map<vector<3>> rp(&residuals[ES_SIZE * i + ES_P]);
            map<vector<3>> rv(&residuals[ES_SIZE * i + ES_V]);
            map<vector<3>> rbg(&residuals[ES_SIZE * i + ES_BG]);
            map<vector<3>> rba(&residuals[ES_SIZE * i + ES_BA]);
            rq = logmap(pose_0[i].q.conjugate() * q);
            rp = p - pose_0[i].p;
            rv = v - motion_0[i].v;
            rbg = bg - motion_0[i].bg;
            rba = ba - motion_0[i].ba;
        }
        if (jacobians) {
            for (size_t i = 0; i < frames.size(); ++i) {
                if (jacobians[5 * i + 0]) {
                    map<matrix<Eigen::Dynamic, 4, true>> dr_dq(jacobians[5 * i + 0], frames.size() * ES_SIZE, 4);
                    const_map<vector<3>> rq(&residuals[ES_SIZE * i + ES_Q]);
                    dr_dq.setZero();
                    dr_dq.block<3, 3>(ES_SIZE * i + ES_Q, 0) = right_jacobian(rq).inverse();
                    dr_dq = sqrt_inv_cov * dr_dq;
                }
                for (size_t k = 1; k < 5; ++k) {
                    if (jacobians[5 * i + k]) {
                        map<matrix<Eigen::Dynamic, 3, true>> dr_dk(jacobians[5 * i + k], frames.size() * ES_SIZE, 3);
                        dr_dk.setZero();
                        dr_dk.block<3, 3>(ES_SIZE * i + k * 3, 0).setIdentity();
                        dr_dk = sqrt_inv_cov * dr_dk;
                    }
                }
            }
        }
        map<vector<>> full_residual(residuals, frames.size() * ES_SIZE);
        full_residual = sqrt_inv_cov * full_residual + infovec;

        return true;
    }

    const std::vector<Frame *> &related_frames() const {
        return frames;
    }

  private:
    std::vector<PoseState> pose_0;
    std::vector<MotionState> motion_0;
    matrix<> sqrt_inv_cov;
    vector<> infovec;
    std::vector<Frame *> frames;
};

} // namespace pvio

#endif // PVIO_MARGINALIZATION_ERROR_COST_H
