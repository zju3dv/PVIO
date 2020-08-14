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
#ifndef PVIO_PAIRWISE_AUGMENTED_PLANE_DISTANCE_ERROR_COST_H
#define PVIO_PAIRWISE_AUGMENTED_PLANE_DISTANCE_ERROR_COST_H

#include <ceres/ceres.h>
#include <pvio/common.h>
#include <pvio/estimation/factor.h>
#include <pvio/estimation/state.h>
#include <pvio/geometry/lie_algebra.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/track.h>

namespace pvio {

class PairwiseAugmentedPlaneDistanceErrorCost : public ceres::SizedCostFunction<1, 4, 3, 4, 3, 3, 1> {
  public:
    PairwiseAugmentedPlaneDistanceErrorCost(const Frame *frame_i, const Frame *frame_j, size_t keypoint_index_i, size_t keypoint_index_j,
                                            double sqrt_inv_cov, double regularization_weight = 1.0) :
        frames({frame_i, frame_j}), keypoint_indices({keypoint_index_i, keypoint_index_j}),
        sqrt_inv_cov(sqrt_inv_cov), regularization_weight(regularization_weight) {
        keypoints[0] = frame_i->get_keypoint(keypoint_index_i);
        keypoints[1] = frame_j->get_keypoint(keypoint_index_j);
    }

    // parameters:
    //  - Device Orientation (quaternion, so3)
    //  - Device Position (vector<3>)
    //  - Plane Normal (vector<2>)
    //  - Plane Distance (double)
    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        matrix<5, 3> A;
        vector<5> b;

        std::array<quaternion, 2> qwc_list;
        std::array<vector<3>, 2> pwc_list;
        std::array<quaternion, 2> qcs_list;
        std::array<matrix<3>, 2> Rsw_list;

        for (size_t i = 0; i < 2; ++i) {
            const_map<quaternion> qwc(parameters[i * 2 + 0]);
            const_map<vector<3>> pwc(parameters[i * 2 + 1]);
            const ExtrinsicParams &camera = frames[i]->camera;
            matrix<3> Rsw = (camera.q_cs.conjugate() * qwc.conjugate()).matrix();
            vector<3> Tsw = -Rsw * pwc - camera.q_cs.conjugate() * camera.p_cs;

            A.row(2 * i + 0) = keypoints[i](0) * Rsw.row(2) - Rsw.row(0);
            A.row(2 * i + 1) = keypoints[i](1) * Rsw.row(2) - Rsw.row(1);
            b(2 * i + 0) = keypoints[i](0) * Tsw(2) - Tsw(0);
            b(2 * i + 1) = keypoints[i](1) * Tsw(2) - Tsw(1);

            qwc_list[i] = qwc;
            pwc_list[i] = pwc;
            qcs_list[i] = camera.q_cs;
            Rsw_list[i] = Rsw;
        }

        const_map<vector<3>> normal(parameters[4]);
        const double &distance(parameters[5][0]);

        A.row(4) = regularization_weight * normal;
        b(4) = regularization_weight * distance;

        matrix<3> ATAinv = (A.transpose() * A).inverse();
        vector<3> ATb = A.transpose() * b;
        vector<3> x = -ATAinv * ATb;

        residuals[0] = normal.dot(x) - distance; // -normal.transpose() * ATAinv * AT * b - b;

        if (jacobians) {
            for (size_t i = 0; i < 2; ++i) {
                matrix<2, 3> Jb;
                Jb << -1.0, 0.0, keypoints[i](0),
                    0.0, -1.0, keypoints[i](1);
                if (double *ptr = jacobians[2 * i + 0]) {
                    map<matrix<1, 4, true>> drdq(ptr);
                    matrix<3> dxdA0 = (b(2 * i + 0) + A.row(2 * i + 0) * x) * ATAinv + (x * A.row(2 * i + 0) * ATAinv).transpose();
                    matrix<3> dxdA1 = (b(2 * i + 1) + A.row(2 * i + 1) * x) * ATAinv + (x * A.row(2 * i + 1) * ATAinv).transpose();
                    matrix<3> dA0dq = qwc_list[i].matrix() * hat(qcs_list[i] * Jb.row(0).transpose());
                    matrix<3> dA1dq = qwc_list[i].matrix() * hat(qcs_list[i] * Jb.row(1).transpose());
                    matrix<3> dxdAdq = dxdA0 * dA0dq + dxdA1 * dA1dq;
                    matrix<3> dxdbdq = ATAinv * A.block<2, 3>(2 * i, 0).transpose() * Jb * qcs_list[i].matrix().transpose() * hat(qwc_list[i].conjugate() * pwc_list[i]);
                    drdq.block<1, 3>(0, 0) = normal.transpose() * (dxdAdq + dxdbdq);
                    drdq(3) = 0;
                    drdq *= sqrt_inv_cov;
                }
                if (double *ptr = jacobians[2 * i + 1]) {
                    map<matrix<1, 3, true>> drdp(ptr);
                    drdp = normal.transpose() * (ATAinv * A.block<2, 3>(2 * i, 0).transpose() * Jb * Rsw_list[i]);
                    drdp *= sqrt_inv_cov;
                }
            }
            if (double *ptr = jacobians[4]) {
                map<matrix<1, 3, true>> drdn(ptr);
                drdn = x.transpose() - regularization_weight * regularization_weight * normal.transpose() * ((distance + normal.dot(x)) * ATAinv + (x * normal.transpose() * ATAinv).transpose());
                drdn *= sqrt_inv_cov;
            }
            if (double *ptr = jacobians[5]) {
                double &drdd(ptr[0]);
                drdd = -(regularization_weight * normal.dot(ATAinv * A.row(4).transpose()) + 1);
                drdd *= sqrt_inv_cov;
            }
        }
        residuals[0] *= sqrt_inv_cov;
        return true;
    }

  private:
    const std::array<const Frame *, 2> frames;
    const std::array<size_t, 2> keypoint_indices;
    std::array<vector<2>, 2> keypoints;
    double sqrt_inv_cov;
    double regularization_weight;
};

} // namespace pvio

#endif // PVIO_PAIRWISE_AUGMENTED_PLANE_DISTANCE_ERROR_COST_H
