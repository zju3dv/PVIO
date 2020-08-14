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
#include <ceres/ceres.h>
#include <pvio/core/plane_extractor.h>
#include <pvio/estimation/ceres/preintegration_error_cost.h>
#include <pvio/estimation/ceres/quaternion_parameterization.h>
#include <pvio/estimation/ceres/reprojection_error_cost.h>
#include <pvio/estimation/ceres/solver_options.h>
#include <pvio/estimation/pnp.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/track.h>
#include <pvio/map/plane.h>

namespace pvio {

void visual_inertial_pnp(Map *map, Frame *frame, Config *config, bool use_inertial) {
    size_t max_iter = config->solver_iteration_limit();
    double max_time = config->solver_time_limit();

    ceres::LocalParameterization *eigen_quaternion = new QuaternionParameterization;
    ceres::LossFunction *cauchy_loss = new ceres::CauchyLoss(1.0);

    ceres::Problem problem;

    problem.AddParameterBlock(frame->pose.q.coeffs().data(), 4, eigen_quaternion);
    problem.AddParameterBlock(frame->pose.p.data(), 3);

    Frame *last_frame = map->last_frame();

    if (use_inertial) {
        problem.AddResidualBlock(new PreIntegrationPriorCost(last_frame, frame),
                                 nullptr,
                                 frame->pose.q.coeffs().data(),
                                 frame->pose.p.data(),
                                 frame->motion.v.data(),
                                 frame->motion.bg.data(),
                                 frame->motion.ba.data());
    }

    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        Track *track = frame->get_track(i);
        if (!track) continue;
        if (!track->has_keypoint(last_frame)) continue;
        if (track->flag(TrackFlag::TF_VALID)) {
#ifdef PVIO_ENABLE_PLANE_CONSTRAINT
            if (track->flag(TrackFlag::TF_PLANE)) {
                // find best plane via reprojection error
                Plane *best_plane = nullptr;
                vector<3> best_plane_point;
                double max_rpe = std::numeric_limits<double>::max();
                vector<3> point = track->get_landmark_point();
                for (size_t j = 0; j < map->plane_num(); ++j) {
                    Plane *plane = map->get_plane(j);
                    const auto &[frame_ref, keypoint_index_ref] = track->first_keypoint();
                    const auto camera_pose = frame_ref->get_pose(frame_ref->camera);
                    vector<3> direction = camera_pose.q * frame_ref->get_keypoint(keypoint_index_ref).homogeneous();
                    if (plane->is_parallel(direction)) continue;
                    vector<3> plane_point = plane->cast_to_point(camera_pose.p, direction);
                    // vector<3> plane_point = plane->point_project_to_plane(point);
                    double rpe = PlaneExtractor::compute_reprojection_error(map, track, plane_point);
                    if (rpe < max_rpe) {
                        best_plane = plane;
                        best_plane_point = plane_point;
                    }
                }
                problem.AddResidualBlock(new PoseOnlyReprojectionXYZErrorCost(best_plane_point, frame, i),
                                         cauchy_loss,
                                         frame->pose.q.coeffs().data(),
                                         frame->pose.p.data());
                continue;
            }
#endif
            problem.AddResidualBlock(new PoseOnlyReprojectionErrorCost(track, frame, i),
                                     cauchy_loss,
                                     frame->pose.q.coeffs().data(),
                                     frame->pose.p.data());
        }
    }

    ceres::Solver::Options solver_options;
    ceres::Solver::Summary solver_summary;
    set_solver_options(solver_options, max_iter, max_time);
    ceres::Solve(solver_options, &problem, &solver_summary);
}

} // namespace pvio
