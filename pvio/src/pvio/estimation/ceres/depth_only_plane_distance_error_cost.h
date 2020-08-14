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
#ifndef PVIO_DEPTH_ONLY_PLANE_DISTANCE_ERROR_COST_H
#define PVIO_DEPTH_ONLY_PLANE_DISTANCE_ERROR_COST_H

#include <ceres/ceres.h>
#include <pvio/common.h>
#include <pvio/estimation/factor.h>
#include <pvio/estimation/state.h>
#include <pvio/geometry/lie_algebra.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/track.h>
#include <pvio/map/plane.h>

namespace pvio {

class DepthOnlyPlaneDistanceErrorCost : public ceres::SizedCostFunction<1, 1> {
  public:
    DepthOnlyPlaneDistanceErrorCost(const Track *track, const Plane *plane, double sqrt_inv_cov) :
        track(track), plane(plane), sqrt_inv_cov(sqrt_inv_cov) {
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        const double &inv_depth(*parameters[0]);

        const auto &[frame_ref, keypoint_index_ref] = track->first_keypoint();
        const PoseState pose_ref_center = frame_ref->get_pose(frame_ref->imu);
        const quaternion &q_ref_center = pose_ref_center.q;
        const vector<3> &p_ref_center = pose_ref_center.p;
        const vector<2> &z_ref = frame_ref->get_keypoint(keypoint_index_ref);

        const ExtrinsicParams &camera_ref = frame_ref->camera;

        vector<3> y_ref = z_ref.homogeneous() / inv_depth;
        vector<3> y_ref_center = camera_ref.q_cs * y_ref + camera_ref.p_cs;
        vector<3> x = q_ref_center * y_ref_center + p_ref_center;

        double &r(residuals[0]);

        const vector<3> &normal = plane->parameter.normal;
        const double distance = plane->parameter.distance;
        r = normal.dot(x) - distance;

        if (jacobians) {
            if (jacobians[0]) {
                double &drdinv_depth(jacobians[0][0]);
                drdinv_depth = -sqrt_inv_cov * normal.dot(q_ref_center * camera_ref.q_cs * y_ref / inv_depth);
            }
        }

        r *= sqrt_inv_cov;

        return true;
    }

  private:
    const Track *track;
    const Plane *plane;
    const double sqrt_inv_cov;
};

} // namespace pvio

#endif // PVIO_DEPTH_ONLY_PLANE_DISTANCE_ERROR_COST_H
