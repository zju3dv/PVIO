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
#ifndef PVIO_REPROJECTION_ERROR_COST_H
#define PVIO_REPROJECTION_ERROR_COST_H

#include <ceres/ceres.h>
#include <pvio/common.h>
#include <pvio/estimation/factor.h>
#include <pvio/estimation/state.h>
#include <pvio/geometry/lie_algebra.h>
#include <pvio/map/frame.h>
#include <pvio/map/track.h>

namespace pvio {

class ReprojectionErrorCost : public Factor::FactorCostFunction, public ceres::SizedCostFunction<2, 4, 3, 4, 3, 1> {
  public:
    ReprojectionErrorCost(const Track *track, const Frame *frame, size_t keypoint_index) :
        track(track), frame(frame), keypoint_index(keypoint_index) {
    }

    void update() override {
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const_map<quaternion> q_tgt_center(parameters[0]);
        const_map<vector<3>> p_tgt_center(parameters[1]);
        const_map<quaternion> q_ref_center(parameters[2]);
        const_map<vector<3>> p_ref_center(parameters[3]);
        const double &inv_depth(*parameters[4]);

        map<vector<2>> r(residuals);

        const auto &[frame_ref, keypoint_index_ref] = track->first_keypoint();

        const vector<2> &z_ref = frame_ref->get_keypoint(keypoint_index_ref);
        const vector<2> &z_tgt = frame->get_keypoint(keypoint_index);

        const ExtrinsicParams &camera_ref = frame_ref->camera;
        const ExtrinsicParams &camera_tgt = frame->camera;

        const matrix<2> &sqrt_inv_cov = frame->sqrt_inv_cov;
        vector<3> y_ref = z_ref.homogeneous() / inv_depth;
        vector<3> y_ref_center = camera_ref.q_cs * y_ref + camera_ref.p_cs;
        vector<3> x = q_ref_center * y_ref_center + p_ref_center;
        vector<3> y_tgt_center = q_tgt_center.conjugate() * (x - p_tgt_center);
        vector<3> y_tgt = camera_tgt.q_cs.conjugate() * (y_tgt_center - camera_tgt.p_cs);
        r = y_tgt.hnormalized() - z_tgt;

        if (jacobians) {
            matrix<2, 3> dr_dy_tgt;
            dr_dy_tgt << 1.0 / y_tgt.z(), 0.0, -y_tgt.x() / (y_tgt.z() * y_tgt.z()),
                0.0, 1.0 / y_tgt.z(), -y_tgt.y() / (y_tgt.z() * y_tgt.z());
            dr_dy_tgt = sqrt_inv_cov * dr_dy_tgt;
            matrix<2, 3> dr_dy_tgt_center;
            matrix<2, 3> dr_dx;
            matrix<2, 3> dr_dy_ref_center;
            matrix<2, 3> dr_dy_ref;
            if (jacobians[0]) {
                dr_dy_tgt_center = dr_dy_tgt * camera_tgt.q_cs.conjugate().matrix(); // dr_dy_tgt_center need to left-multiply sqrt_inv_cov
            }
            if (jacobians[1] || jacobians[3]) {
                if (jacobians[0]) {
                    dr_dx = dr_dy_tgt_center * q_tgt_center.conjugate().matrix();
                } else {
                    dr_dx = dr_dy_tgt * (camera_tgt.q_cs.conjugate() * q_tgt_center.conjugate()).matrix();
                }
            }
            if (jacobians[2] || jacobians[4]) {
                if (jacobians[1] || jacobians[3]) {
                    dr_dy_ref_center = dr_dx * q_ref_center.matrix();
                } else if (jacobians[0]) {
                    dr_dy_ref_center = dr_dy_tgt_center * (q_tgt_center.conjugate() * q_ref_center).matrix();
                } else {
                    dr_dy_ref_center = dr_dy_tgt * (camera_tgt.q_cs.conjugate() * q_tgt_center.conjugate() * q_ref_center).matrix();
                }
            }
            if (jacobians[0]) {
                map<matrix<2, 4, true>> dr_dq_tgt(jacobians[0]);
                dr_dq_tgt.block<2, 3>(0, 0) = dr_dy_tgt_center * hat(y_tgt_center);
                dr_dq_tgt.col(3).setZero();
            }
            if (jacobians[1]) {
                map<matrix<2, 3, true>> dr_dp_tgt(jacobians[1]);
                dr_dp_tgt = -dr_dx;
            }
            if (jacobians[2]) {
                map<matrix<2, 4, true>> dr_dq_ref(jacobians[2]);
                dr_dq_ref.block<2, 3>(0, 0) = -dr_dy_ref_center * hat(y_ref_center);
                dr_dq_ref.col(3).setZero();
            }
            if (jacobians[3]) {
                map<matrix<2, 3, true>> dr_dp_ref(jacobians[3]);
                dr_dp_ref = dr_dx;
            }
            if (jacobians[4]) {
                map<matrix<2, 1, true>> dr_dinv_depth(jacobians[4]);
                dr_dinv_depth = -dr_dy_ref_center * camera_ref.q_cs.matrix() * y_ref / inv_depth;
            }
        }

        r = sqrt_inv_cov * r;

        return true;
    };

  private:
    const Track *track;
    const Frame *frame;
    const size_t keypoint_index;
};

class PoseOnlyReprojectionErrorCost : public ceres::SizedCostFunction<2, 4, 3> {
  public:
    PoseOnlyReprojectionErrorCost(const Track *track, const Frame *frame, size_t keypoint_index) :
        error(track, frame, keypoint_index), track(track) {
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        std::array<const double *, 5> params = {
            parameters[0],
            parameters[1],
            track->first_frame()->pose.q.coeffs().data(),
            track->first_frame()->pose.p.data(),
            &(track->landmark.inv_depth)};
        if (jacobians) {
            std::array<double *, 5> jacobs = {
                jacobians[0],
                jacobians[1],
                nullptr,
                nullptr,
                nullptr};
            return error.Evaluate(params.data(), residuals, jacobs.data());
        } else {
            return error.Evaluate(params.data(), residuals, nullptr);
        }
    }

  private:
    ReprojectionErrorCost error;
    const Track *track;
};

class PoseOnlyReprojectionXYZErrorCost : public ceres::SizedCostFunction<2, 4, 3> {
  public:
    PoseOnlyReprojectionXYZErrorCost(const vector<3> &point, const Frame *frame, const size_t keypoint_index) :
        frame(frame), point(point), keypoint_index(keypoint_index) {
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const_map<quaternion> q_tgt_center(parameters[0]);
        const_map<vector<3>> p_tgt_center(parameters[1]);
        const ExtrinsicParams &camera_tgt = frame->camera;
        const vector<2> &z_tgt = frame->get_keypoint(keypoint_index);

        vector<3> y_tgt_center = q_tgt_center.conjugate() * (point - p_tgt_center);
        vector<3> y_tgt = camera_tgt.q_cs.conjugate() * (y_tgt_center - camera_tgt.p_cs);

        const matrix<2> &sqrt_inv_cov = frame->sqrt_inv_cov;

        map<vector<2>> r(residuals);
        r = y_tgt.hnormalized() - z_tgt;
        if (jacobians) {
            matrix<2, 3> dr_dy_tgt;
            dr_dy_tgt << 1.0 / y_tgt.z(), 0.0, -y_tgt.x() / (y_tgt.z() * y_tgt.z()),
                0.0, 1.0 / y_tgt.z(), -y_tgt.y() / (y_tgt.z() * y_tgt.z());
            dr_dy_tgt = sqrt_inv_cov * dr_dy_tgt;
            if (jacobians[0]) {
                map<matrix<2, 4, true>> dr_dq_tgt(jacobians[0]);
                dr_dq_tgt.block<2, 3>(0, 0) = dr_dy_tgt * camera_tgt.q_cs.conjugate().matrix() * hat(y_tgt_center);
                dr_dq_tgt.col(3).setZero();
            }
            if (jacobians[1]) {
                map<matrix<2, 3, true>> dr_dp_tgt(jacobians[1]);
                dr_dp_tgt = -dr_dy_tgt * (camera_tgt.q_cs.conjugate() * q_tgt_center.conjugate()).matrix();;
            }
        }

        r = sqrt_inv_cov * r;

        return true;
    }

  private:
    const vector<3> point;
    const Frame *frame;
    const size_t keypoint_index;
};

} // namespace pvio

#endif // PVIO_REPROJECTION_ERROR_COST_H
