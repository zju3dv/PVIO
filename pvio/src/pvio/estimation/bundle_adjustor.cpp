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
#include <pvio/estimation/bundle_adjustor.h>
#include <pvio/estimation/ceres/marginalization_error_cost.h>
#include <pvio/estimation/ceres/plane_distance_error_cost.h>
#include <pvio/estimation/ceres/augmented_plane_distance_error_cost.h>
#include <pvio/estimation/ceres/pairwise_augmented_plane_distance_error_cost.h>
#include <pvio/estimation/ceres/depth_only_plane_distance_error_cost.h>
#include <pvio/estimation/ceres/preintegration_error_cost.h>
#include <pvio/estimation/ceres/quaternion_parameterization.h>
#include <pvio/estimation/ceres/reprojection_error_cost.h>
#include <pvio/estimation/ceres/solver_options.h>
#include <pvio/estimation/factor.h>
#include <pvio/forensics.h>
#include <pvio/geometry/stereo.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/plane.h>
#include <pvio/map/track.h>
#include <pvio/utility/unique_timer.h>

namespace pvio {

struct BundleAdjustorIterationCallback : public ceres::IterationCallback {
    std::vector<Factor::FactorCostFunction *> cost_functions;

    ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {
        for (Factor::FactorCostFunction *cost : cost_functions) {
            cost->update();
        }
        return ceres::SOLVER_CONTINUE;
    }
};

struct BundleAdjustor::BundleAdjustorSolver {
    std::unique_ptr<ceres::LossFunction> cauchy_loss;
    std::unique_ptr<ceres::LocalParameterization> quaternion_parameterization;
    std::unique_ptr<ceres::LocalParameterization> normal_parameterization;

    BundleAdjustorSolver() {
        cauchy_loss = std::make_unique<ceres::CauchyLoss>(1.0);
        quaternion_parameterization = std::make_unique<QuaternionParameterization>();
        normal_parameterization = std::make_unique<ceres::HomogeneousVectorParameterization>(3);
    }

    bool solve(Map *map, Config *config, bool use_inertial) {
        size_t max_iter = config->solver_iteration_limit();
        double max_time = config->solver_time_limit();

        BundleAdjustorIterationCallback iteration_callback;

        ceres::Problem::Options problem_options;
        problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        ceres::Problem problem(problem_options);

        for (size_t i = 0; i < map->frame_num(); ++i) {
            Frame *frame = map->get_frame(i);
            problem.AddParameterBlock(frame->pose.q.coeffs().data(), 4, quaternion_parameterization.get());
            problem.AddParameterBlock(frame->pose.p.data(), 3);
            if (frame->flag(FrameFlag::FF_FIX_POSE)) {
                problem.SetParameterBlockConstant(frame->pose.q.coeffs().data());
                problem.SetParameterBlockConstant(frame->pose.p.data());
            }
            if (use_inertial) {
                problem.AddParameterBlock(frame->motion.v.data(), 3);
                problem.AddParameterBlock(frame->motion.bg.data(), 3);
                problem.AddParameterBlock(frame->motion.ba.data(), 3);
            }
        }

#ifdef PVIO_ENABLE_PLANE_CONSTRAINT
        std::unordered_set<Track *> visited_tracks;
        for (size_t i = 0; i < map->frame_num(); ++i) {
            Frame *frame = map->get_frame(i);
            for (size_t j = 0; j < frame->keypoint_num(); ++j) {
                Track *track = frame->get_track(j);
                if (!track) continue;
                if (!track->flag(TrackFlag::TF_VALID)) continue;
                if (track->flag(TrackFlag::TF_PLANE)) continue;
                if (visited_tracks.count(track) > 0) continue;
                visited_tracks.insert(track);
                problem.AddParameterBlock(&(track->landmark.inv_depth), 1);
            }
        }
        for (size_t i = 0; i < map->plane_num(); ++i) {
            Plane *plane = map->get_plane(i);
            problem.AddParameterBlock(plane->parameter.normal.data(), 3, normal_parameterization.get());
            problem.AddParameterBlock(&(plane->parameter.distance), 1);
            problem.SetParameterBlockConstant(plane->parameter.normal.data());
            problem.SetParameterBlockConstant(&(plane->parameter.distance));
        }
#else
        std::unordered_set<Track *> visited_tracks;
        for (size_t i = 0; i < map->frame_num(); ++i) {
            Frame *frame = map->get_frame(i);
            for (size_t j = 0; j < frame->keypoint_num(); ++j) {
                Track *track = frame->get_track(j);
                if (!track) continue;
                if (!track->flag(TrackFlag::TF_VALID)) continue;
                if (visited_tracks.count(track) > 0) continue;
                visited_tracks.insert(track);
                problem.AddParameterBlock(&(track->landmark.inv_depth), 1);
            }
        }
#endif

        if (map->get_marginalization_factor()) {
            MarginalizationErrorCost *marcost = map->get_marginalization_factor()->get_cost_function<MarginalizationErrorCost>();
            std::vector<double *> params;
            for (size_t i = 0; i < marcost->related_frames().size(); ++i) {
                Frame *frame = marcost->related_frames()[i];
                params.emplace_back(frame->pose.q.coeffs().data());
                params.emplace_back(frame->pose.p.data());
                params.emplace_back(frame->motion.v.data());
                params.emplace_back(frame->motion.bg.data());
                params.emplace_back(frame->motion.ba.data());
            }
            problem.AddResidualBlock(marcost, nullptr, params);
            iteration_callback.cost_functions.push_back(marcost);
        }

#ifdef PVIO_ENABLE_PLANE_CONSTRAINT
        for (size_t i = 0; i < map->frame_num(); ++i) {
            Frame *frame = map->get_frame(i);
            for (size_t j = 0; j < frame->keypoint_num(); ++j) {
                Track *track = frame->get_track(j);
                if (!track) continue;
                if (!track->flag(TrackFlag::TF_VALID)) continue;
                if (track->flag(TrackFlag::TF_PLANE)) continue;
                if (frame == track->first_frame()) continue;
                ReprojectionErrorCost *rpecost = frame->get_reprojection_factor(j)->get_cost_function<ReprojectionErrorCost>();
                problem.AddResidualBlock(
                    rpecost,
                    cauchy_loss.get(),
                    frame->pose.q.coeffs().data(),
                    frame->pose.p.data(),
                    track->first_frame()->pose.q.coeffs().data(),
                    track->first_frame()->pose.p.data(),
                    &(track->landmark.inv_depth));
                iteration_callback.cost_functions.push_back(rpecost);
            }
        }
        std::vector<std::unique_ptr<ceres::CostFunction>> plane_factors;
        for (size_t i = 0; i < map->plane_num(); ++i) {
            Plane *plane = map->get_plane(i);
            if (plane->tracks.size() < 20) {
                for (Track *track : plane->tracks) {
                    for (auto [frame, keypoint_index] : track->keypoint_map()) {
                        if (frame == track->first_frame()) continue;
                        ReprojectionErrorCost *rpecost = frame->get_reprojection_factor(keypoint_index)->get_cost_function<ReprojectionErrorCost>();
                        problem.AddResidualBlock(
                            rpecost,
                            cauchy_loss.get(),
                            frame->pose.q.coeffs().data(),
                            frame->pose.p.data(),
                            track->first_frame()->pose.q.coeffs().data(),
                            track->first_frame()->pose.p.data(),
                            &(track->landmark.inv_depth));
                    }
                }
            } else {
                double plane_distance_sqrt_inv_cov = sqrt(1.0 / config->plane_distance_cov());
                for (Track *track : plane->tracks) {
                    if (track->landmark.plane_id != plane->id()) continue; // if not the best plane, skip
                    std::unique_ptr<AugmentedPlaneDistanceErrorCost> factor = std::make_unique<AugmentedPlaneDistanceErrorCost>(track, plane_distance_sqrt_inv_cov);
                    std::vector<double *> factor_parameters;
                    for (auto [frame, keypoint_index] : track->keypoint_map()) {
                        factor_parameters.push_back(frame->pose.q.coeffs().data());
                        factor_parameters.push_back(frame->pose.p.data());
                    }
                    factor_parameters.push_back(plane->parameter.normal.data());
                    factor_parameters.push_back(&(plane->parameter.distance));
                    problem.AddResidualBlock(factor.get(), cauchy_loss.get(), factor_parameters);
                    plane_factors.emplace_back(std::move(factor));
                }
            }
        }

#else
        for (size_t i = 0; i < map->frame_num(); ++i) {
            Frame *frame = map->get_frame(i);
            for (size_t j = 0; j < frame->keypoint_num(); ++j) {
                Track *track = frame->get_track(j);
                if (!track) continue;
                if (!track->flag(TrackFlag::TF_VALID)) continue;
                if (frame == track->first_frame()) continue;
                ReprojectionErrorCost *rpecost = frame->get_reprojection_factor(j)->get_cost_function<ReprojectionErrorCost>();
                problem.AddResidualBlock(
                    rpecost,
                    cauchy_loss.get(),
                    frame->pose.q.coeffs().data(),
                    frame->pose.p.data(),
                    track->first_frame()->pose.q.coeffs().data(),
                    track->first_frame()->pose.p.data(),
                    &(track->landmark.inv_depth));
                iteration_callback.cost_functions.push_back(rpecost);
            }
        }
#endif

        if (use_inertial) {
            for (size_t j = 1; j < map->frame_num(); ++j) {
                Frame *frame_i = map->get_frame(j - 1);
                Frame *frame_j = map->get_frame(j);
                if (frame_j->preintegration.integrate(frame_j->image->t, frame_i->motion.bg, frame_i->motion.ba, true, true)) {
                    PreIntegrationErrorCost *piecost = frame_j->get_preintegration_factor()->get_cost_function<PreIntegrationErrorCost>();
                    problem.AddResidualBlock(
                        piecost,
                        nullptr,
                        frame_i->pose.q.coeffs().data(),
                        frame_i->pose.p.data(),
                        frame_i->motion.v.data(),
                        frame_i->motion.bg.data(),
                        frame_i->motion.ba.data(),
                        frame_j->pose.q.coeffs().data(),
                        frame_j->pose.p.data(),
                        frame_j->motion.v.data(),
                        frame_j->motion.bg.data(),
                        frame_j->motion.ba.data());
                    iteration_callback.cost_functions.push_back(piecost);
                }
            }
        }

        ceres::Solver::Options solver_options;
        ceres::Solver::Summary solver_summary;
        set_solver_options(solver_options, max_iter, max_time);
        solver_options.update_state_every_iteration = true;
        solver_options.callbacks.push_back(&iteration_callback);
        ceres::Solve(solver_options, &problem, &solver_summary);

#if defined(PVIO_ENABLE_PLANE_CONSTRAINT)
        for (size_t i = 0; i < map->track_num(); ++i) {
            Track *track = map->get_track(i);
            if (!track->flag(TrackFlag::TF_PLANE)) continue;
            if (track->keypoint_num() < 2) continue;
            vector<3> p;
            if (track->life > 10 && PlaneExtractor::enough_baseline(track) && track->try_triangulate(p)) {
                bool is_point_in_plane = false;
                for (size_t j = 0; j < map->plane_num(); ++j) {
                    Plane *plane = map->get_plane(j);
                    if (plane->tracks.count(track) == 0) continue;
                    if (std::abs(plane->parameter.normal.dot(p) - plane->parameter.distance) > 0.1) {
                        plane->tracks.erase(track);
                    } else {
                        is_point_in_plane = true;
                    }
                }
                if (!is_point_in_plane) {
                    track->flag(TrackFlag::TF_PLANE) = false;
                    track->flag(TrackFlag::TF_VALID) = true;
                    track->set_landmark_point(p);
                }
            }
        }
#endif

        for (size_t i = 0; i < map->track_num(); ++i) {
            Track *track = map->get_track(i);
            if (!track->flag(TrackFlag::TF_VALID) && !track->flag(TrackFlag::TF_PLANE)) continue;
            const vector<3> &x = track->get_landmark_point();
            double quality = 0.0;
            double quality_num = 0.0;
            for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
                PoseState pose = frame->get_pose(frame->camera);
                vector<3> y = pose.q.conjugate() * (x - pose.p);
                if (y.z() <= 1.0e-3 || y.z() > 50) {
                    track->flag(TrackFlag::TF_VALID) = false;
                    track->flag(TrackFlag::TF_PLANE) = false;
                    break;
                }
                quality += (apply_k(y.hnormalized(), frame->K) - apply_k(frame->get_keypoint(keypoint_index), frame->K)).norm();
                quality_num += 1.0;
            }
            if (!track->flag(TrackFlag::TF_VALID)) continue;
            track->landmark.quality = quality / std::max(quality_num, 1.0);
        }

        return solver_summary.IsSolutionUsable();
    }
};

BundleAdjustor::BundleAdjustor() {
    solver = std::make_unique<BundleAdjustorSolver>();
}

BundleAdjustor::~BundleAdjustor() = default;

bool BundleAdjustor::solve(Map *map, Config *config, bool use_inertial) {
    auto ba_timer = make_timer([](double t) {
        forensics(bundle_adjustor_solve_time, time) {
            static double avg_time = 0;
            static double avg_count = 0;
            avg_time = (avg_time * avg_count + t) / (avg_count + 1);
            avg_count += 1.0;
            time = avg_time;
        }
    });
    return solver->solve(map, config, use_inertial);
}

double BundleAdjustor::compute_reprojection_error(Map *map) {
    double quality = 0.0;
    double quality_num = 0.0;
    for (size_t i = 0; i < map->track_num(); ++i) {
        Track *track = map->get_track(i);
        if (!track->flag(TrackFlag::TF_VALID)) continue;
        const vector<3> &x = track->get_landmark_point();
        for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
            PoseState pose = frame->get_pose(frame->camera);
            vector<3> y = pose.q.conjugate() * (x - pose.p);
            quality += (apply_k(y.hnormalized(), frame->K) - apply_k(frame->get_keypoint(keypoint_index), frame->K)).norm();
            quality_num += 1.0;
        }
    }
    return quality / std::max(quality_num, 1.0);
}

struct LandmarkInfo {
    LandmarkInfo() {
        mat = 0;
        vec = 0;
    }
    double mat;
    double vec;
    std::unordered_map<size_t, matrix<1, 6>> h;
};

void BundleAdjustor::marginalize_frame(Map *map, size_t index) {
    auto ba_timer = make_timer([](double t) {
        forensics(bundle_adjustor_marginalization_time, time) {
            time = t;
        }
    });
    matrix<> pose_motion_infomat;
    vector<> pose_motion_infovec;
    std::map<Track *, LandmarkInfo, compare<Track *>> landmark_info;

    pose_motion_infomat.resize(map->frame_num() * ES_SIZE, map->frame_num() * ES_SIZE);
    pose_motion_infovec.resize(map->frame_num() * ES_SIZE);
    pose_motion_infomat.setZero();
    pose_motion_infovec.setZero();

    std::unordered_map<Frame *, size_t> frame_indices;
    for (size_t i = 0; i < map->frame_num(); ++i) {
        frame_indices[map->get_frame(i)] = i;
    }

    /* scope: marginalization factor */
    if (map->get_marginalization_factor()) {
        MarginalizationErrorCost *marcost = map->get_marginalization_factor()->get_cost_function<MarginalizationErrorCost>();
        const std::vector<Frame *> frames = marcost->related_frames();
        std::vector<const double *> marparameters(frames.size() * 5);
        std::vector<double *> marjacobian_ptrs(frames.size() * 5);
        std::vector<matrix<Eigen::Dynamic, Eigen::Dynamic, true>> marjacobians(frames.size() * 5);
        for (size_t i = 0; i < frames.size(); ++i) {
            marparameters[5 * i + 0] = frames[i]->pose.q.coeffs().data();
            marparameters[5 * i + 1] = frames[i]->pose.p.data();
            marparameters[5 * i + 2] = frames[i]->motion.v.data();
            marparameters[5 * i + 3] = frames[i]->motion.bg.data();
            marparameters[5 * i + 4] = frames[i]->motion.ba.data();
            marjacobians[5 * i + 0].resize(frames.size() * ES_SIZE, 4);
            marjacobians[5 * i + 1].resize(frames.size() * ES_SIZE, 3);
            marjacobians[5 * i + 2].resize(frames.size() * ES_SIZE, 3);
            marjacobians[5 * i + 3].resize(frames.size() * ES_SIZE, 3);
            marjacobians[5 * i + 4].resize(frames.size() * ES_SIZE, 3);
            marjacobian_ptrs[5 * i + 0] = marjacobians[5 * i + 0].data();
            marjacobian_ptrs[5 * i + 1] = marjacobians[5 * i + 1].data();
            marjacobian_ptrs[5 * i + 2] = marjacobians[5 * i + 2].data();
            marjacobian_ptrs[5 * i + 3] = marjacobians[5 * i + 3].data();
            marjacobian_ptrs[5 * i + 4] = marjacobians[5 * i + 4].data();
        }
        vector<> marresidual;
        marresidual.resize(frames.size() * ES_SIZE);
        marcost->Evaluate(marparameters.data(), marresidual.data(), marjacobian_ptrs.data());

        std::vector<matrix<>> state_jacobians(frames.size());
        for (size_t i = 0; i < frames.size(); ++i) {
            size_t frame_index = frame_indices[frames[i]];
            matrix<> &dr_ds = state_jacobians[frame_index];
            dr_ds.resize(frames.size() * ES_SIZE, ES_SIZE);
            dr_ds.block(0, ES_Q, frames.size() * ES_SIZE, 3) = marjacobians[5 * i + 0].leftCols(3);
            dr_ds.block(0, ES_P, frames.size() * ES_SIZE, 3) = marjacobians[5 * i + 1];
            dr_ds.block(0, ES_V, frames.size() * ES_SIZE, 3) = marjacobians[5 * i + 2];
            dr_ds.block(0, ES_BG, frames.size() * ES_SIZE, 3) = marjacobians[5 * i + 3];
            dr_ds.block(0, ES_BA, frames.size() * ES_SIZE, 3) = marjacobians[5 * i + 4];
        }
        for (size_t i = 0; i < frames.size(); ++i) {
            for (size_t j = 0; j < frames.size(); ++j) {
                pose_motion_infomat.block<ES_SIZE, ES_SIZE>(ES_SIZE * i, ES_SIZE * j) += state_jacobians[i].transpose() * state_jacobians[j];
            }
            pose_motion_infovec.segment<ES_SIZE>(ES_SIZE * i) += state_jacobians[i].transpose() * marresidual;
        }
    }

    /* scope: preintegration factor */
    for (size_t j = index; j <= index + 1; ++j) {
        if (j == 0) continue;
        if (j >= map->frame_num()) continue;
        size_t i = j - 1;
        Frame *frame_i = map->get_frame(i);
        Frame *frame_j = map->get_frame(j);
        if (!frame_j->get_preintegration_factor()) continue;
        PreIntegrationErrorCost *picost = frame_j->get_preintegration_factor()->get_cost_function<PreIntegrationErrorCost>();
        std::array<const double *, 10> piparameters = {
            frame_i->pose.q.coeffs().data(), frame_i->pose.p.data(), frame_i->motion.v.data(), frame_i->motion.bg.data(), frame_i->motion.ba.data(),
            frame_j->pose.q.coeffs().data(), frame_j->pose.p.data(), frame_j->motion.v.data(), frame_j->motion.bg.data(), frame_j->motion.ba.data()};
        vector<ES_SIZE> piresidual;
        matrix<ES_SIZE, 4, true> dr_dqi, dr_dqj;
        matrix<ES_SIZE, 3, true> dr_dpi, dr_dpj;
        matrix<ES_SIZE, 3, true> dr_dvi, dr_dvj;
        matrix<ES_SIZE, 3, true> dr_dbgi, dr_dbgj;
        matrix<ES_SIZE, 3, true> dr_dbai, dr_dbaj;
        std::array<double *, 10> pijacobians = {
            dr_dqi.data(), dr_dpi.data(), dr_dvi.data(), dr_dbgi.data(), dr_dbai.data(),
            dr_dqj.data(), dr_dpj.data(), dr_dvj.data(), dr_dbgj.data(), dr_dbaj.data()};
        picost->Evaluate(piparameters.data(), piresidual.data(), pijacobians.data());
        matrix<ES_SIZE, ES_SIZE * 2> dr_dstates;
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 0 + ES_Q) = dr_dqi.block<ES_SIZE, 3>(0, 0);
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 0 + ES_P) = dr_dpi;
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 0 + ES_V) = dr_dvi;
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 0 + ES_BG) = dr_dbgi;
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 0 + ES_BA) = dr_dbai;
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 1 + ES_Q) = dr_dqj.block<ES_SIZE, 3>(0, 0);
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 1 + ES_P) = dr_dpj;
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 1 + ES_V) = dr_dvj;
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 1 + ES_BG) = dr_dbgj;
        dr_dstates.block<ES_SIZE, 3>(0, ES_SIZE * 1 + ES_BA) = dr_dbaj;
        pose_motion_infomat.block<ES_SIZE * 2, ES_SIZE * 2>(ES_SIZE * i, ES_SIZE * i) += dr_dstates.transpose() * dr_dstates;
        pose_motion_infovec.segment<ES_SIZE * 2>(ES_SIZE * i) += dr_dstates.transpose() * piresidual;
    }

    /* scope: reprojection error factor */
    Frame *frame_victim = map->get_frame(index);
    for (size_t j = 0; j < frame_victim->keypoint_num(); ++j) {
        Track *track = frame_victim->get_track(j);
        if (!track || !track->flag(TrackFlag::TF_VALID)) continue;
        if (track->flag(TrackFlag::TF_PLANE)) continue;
        Frame *frame_ref = track->first_frame();
        size_t frame_index_ref = frame_indices.at(frame_ref);
        for (const auto &[frame_tgt, keypoint_index] : track->keypoint_map()) {
            if (frame_tgt == frame_ref) continue;
            if (frame_indices.count(frame_tgt) == 0) continue;
            size_t frame_index_tgt = frame_indices.at(frame_tgt);
            ReprojectionErrorCost *rpcost = frame_tgt->get_reprojection_factor(keypoint_index)->get_cost_function<ReprojectionErrorCost>();
            std::array<const double *, 5> rpparameters = {
                frame_tgt->pose.q.coeffs().data(),
                frame_tgt->pose.p.data(),
                frame_ref->pose.q.coeffs().data(),
                frame_ref->pose.p.data(),
                &(track->landmark.inv_depth)};
            vector<2> rpresidual;
            matrix<2, 4, true> dr_dq_tgt;
            matrix<2, 3, true> dr_dp_tgt;
            matrix<2, 4, true> dr_dq_ref;
            matrix<2, 3, true> dr_dp_ref;
            vector<2> dr_dinv_depth;
            std::array<double *, 5> rpjacobians = {
                dr_dq_tgt.data(),
                dr_dp_tgt.data(),
                dr_dq_ref.data(),
                dr_dp_ref.data(),
                dr_dinv_depth.data()};
            rpcost->Evaluate(rpparameters.data(), rpresidual.data(), rpjacobians.data());
            matrix<2, 3, true> dr_dq_tgt_local = dr_dq_tgt.block<2, 3>(0, 0);
            matrix<2, 3, true> dr_dq_ref_local = dr_dq_ref.block<2, 3>(0, 0);

            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_tgt + ES_Q, ES_SIZE * frame_index_tgt + ES_Q) += dr_dq_tgt_local.transpose() * dr_dq_tgt_local;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_tgt + ES_P, ES_SIZE * frame_index_tgt + ES_P) += dr_dp_tgt.transpose() * dr_dp_tgt;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_tgt + ES_Q, ES_SIZE * frame_index_tgt + ES_P) += dr_dq_tgt_local.transpose() * dr_dp_tgt;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_tgt + ES_P, ES_SIZE * frame_index_tgt + ES_Q) += dr_dp_tgt.transpose() * dr_dq_tgt_local;

            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_ref + ES_Q, ES_SIZE * frame_index_tgt + ES_Q) += dr_dq_ref_local.transpose() * dr_dq_tgt_local;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_ref + ES_P, ES_SIZE * frame_index_tgt + ES_P) += dr_dp_ref.transpose() * dr_dp_tgt;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_ref + ES_Q, ES_SIZE * frame_index_tgt + ES_P) += dr_dq_ref_local.transpose() * dr_dp_tgt;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_ref + ES_P, ES_SIZE * frame_index_tgt + ES_Q) += dr_dp_ref.transpose() * dr_dq_tgt_local;

            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_tgt + ES_Q, ES_SIZE * frame_index_ref + ES_Q) += dr_dq_tgt_local.transpose() * dr_dq_ref_local;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_tgt + ES_P, ES_SIZE * frame_index_ref + ES_P) += dr_dp_tgt.transpose() * dr_dp_ref;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_tgt + ES_Q, ES_SIZE * frame_index_ref + ES_P) += dr_dq_tgt_local.transpose() * dr_dp_ref;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_tgt + ES_P, ES_SIZE * frame_index_ref + ES_Q) += dr_dp_tgt.transpose() * dr_dq_ref_local;

            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_ref + ES_Q, ES_SIZE * frame_index_ref + ES_Q) += dr_dq_ref_local.transpose() * dr_dq_ref_local;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_ref + ES_P, ES_SIZE * frame_index_ref + ES_P) += dr_dp_ref.transpose() * dr_dp_ref;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_ref + ES_Q, ES_SIZE * frame_index_ref + ES_P) += dr_dq_ref_local.transpose() * dr_dp_ref;
            pose_motion_infomat.block<3, 3>(ES_SIZE * frame_index_ref + ES_P, ES_SIZE * frame_index_ref + ES_Q) += dr_dp_ref.transpose() * dr_dq_ref_local;

            pose_motion_infovec.segment<3>(ES_SIZE * frame_index_tgt + ES_Q) += dr_dq_tgt_local.transpose() * rpresidual;
            pose_motion_infovec.segment<3>(ES_SIZE * frame_index_tgt + ES_P) += dr_dp_tgt.transpose() * rpresidual;
            pose_motion_infovec.segment<3>(ES_SIZE * frame_index_ref + ES_Q) += dr_dq_ref_local.transpose() * rpresidual;
            pose_motion_infovec.segment<3>(ES_SIZE * frame_index_ref + ES_P) += dr_dp_ref.transpose() * rpresidual;

            LandmarkInfo &linfo = landmark_info[track];

            linfo.mat += dr_dinv_depth.transpose() * dr_dinv_depth;
            linfo.vec += dr_dinv_depth.transpose() * rpresidual;
            if (linfo.h.count(frame_index_tgt) == 0) {
                linfo.h[frame_index_tgt].setZero();
            }
            if (linfo.h.count(frame_index_ref) == 0) {
                linfo.h[frame_index_ref].setZero();
            }
            /* scope */ {
                matrix<1, 6> &h = linfo.h.at(frame_index_tgt);
                h.segment<3>(ES_Q - ES_Q) += dr_dinv_depth.transpose() * dr_dq_tgt_local;
                h.segment<3>(ES_P - ES_Q) += dr_dinv_depth.transpose() * dr_dp_tgt;
            }
            /* scope */ {
                matrix<1, 6> &h = linfo.h.at(frame_index_ref);
                h.segment<3>(ES_Q - ES_Q) += dr_dinv_depth.transpose() * dr_dq_ref_local;
                h.segment<3>(ES_P - ES_Q) += dr_dinv_depth.transpose() * dr_dp_ref;
            }
        }
    }

    /* scope: marginalize landmarks */
    for (const auto &[track, info] : landmark_info) {
        double inv_infomat = 1.0 / info.mat;
        if (!std::isfinite(inv_infomat)) continue;
        for (const auto &[frame_index_i, h_i] : info.h) {
            for (const auto &[frame_index_j, h_j] : info.h) {
                pose_motion_infomat.block<6, 6>(ES_SIZE * frame_index_i + ES_Q, ES_SIZE * frame_index_j + ES_Q) -= h_i.transpose() * inv_infomat * h_j;
            }
            pose_motion_infovec.segment<6>(ES_SIZE * frame_index_i + ES_Q) -= h_i.transpose() * inv_infomat * info.vec;
        }
    }

    /* scope: marginalize the corresponding frame */ {
        matrix<15, 15> inv_infomat = pose_motion_infomat.block<ES_SIZE, ES_SIZE>(ES_SIZE * index, ES_SIZE * index).inverse();
        matrix<> complement_infomat;
        vector<> complement_infovec;
        complement_infomat.resize(ES_SIZE * (map->frame_num() - 1), ES_SIZE * (map->frame_num() - 1));
        complement_infovec.resize(ES_SIZE * (map->frame_num() - 1));
        complement_infomat.setZero();
        complement_infovec.setZero();

        if (index > 0) {
            matrix<> complement_infomat_block = pose_motion_infomat.block(0, 0, ES_SIZE * index, ES_SIZE * index);
            vector<> complement_infovec_segment = pose_motion_infovec.segment(0, ES_SIZE * index);
            complement_infomat_block -= pose_motion_infomat.block(0, ES_SIZE * index, ES_SIZE * index, ES_SIZE) * inv_infomat * pose_motion_infomat.block(ES_SIZE * index, 0, ES_SIZE, ES_SIZE * index);
            complement_infovec_segment -= pose_motion_infomat.block(0, ES_SIZE * index, ES_SIZE * index, ES_SIZE) * inv_infomat * pose_motion_infovec.segment(ES_SIZE * index, ES_SIZE);
            complement_infomat.block(0, 0, ES_SIZE * index, ES_SIZE * index) = complement_infomat_block;
            complement_infovec.segment(0, ES_SIZE * index) = complement_infovec_segment;
        }
        if (index < map->frame_num() - 1) {
            matrix<> complement_infomat_block = pose_motion_infomat.block(ES_SIZE * (index + 1), ES_SIZE * (index + 1), ES_SIZE * (map->frame_num() - 1 - index), ES_SIZE * (map->frame_num() - 1 - index));
            vector<> complement_infovec_segment = pose_motion_infovec.segment(ES_SIZE * (index + 1), ES_SIZE * (map->frame_num() - 1 - index));
            complement_infomat_block -= pose_motion_infomat.block(ES_SIZE * (index + 1), ES_SIZE * index, ES_SIZE * (map->frame_num() - 1 - index), ES_SIZE) * inv_infomat * pose_motion_infomat.block(ES_SIZE * index, ES_SIZE * (index + 1), ES_SIZE, ES_SIZE * (map->frame_num() - 1 - index));
            complement_infovec_segment -= pose_motion_infomat.block(ES_SIZE * (index + 1), ES_SIZE * index, ES_SIZE * (map->frame_num() - 1 - index), ES_SIZE) * inv_infomat * pose_motion_infovec.segment(ES_SIZE * index, ES_SIZE);
            complement_infomat.block(ES_SIZE * index, ES_SIZE * index, ES_SIZE * (map->frame_num() - 1 - index), ES_SIZE * (map->frame_num() - 1 - index)) = complement_infomat_block;
            complement_infovec.segment(ES_SIZE * index, ES_SIZE * (map->frame_num() - 1 - index)) = complement_infovec_segment;
        }
        if (index > 0 && index < map->frame_num() - 1) {
            matrix<> complement_infomat_block = pose_motion_infomat.block(0, ES_SIZE * (index + 1), ES_SIZE * index, ES_SIZE * (map->frame_num() - 1 - index));
            complement_infomat_block -= pose_motion_infomat.block(0, ES_SIZE * index, ES_SIZE * index, ES_SIZE) * inv_infomat * pose_motion_infomat.block(ES_SIZE * index, ES_SIZE * (index + 1), ES_SIZE, ES_SIZE * (map->frame_num() - 1 - index));
            complement_infomat.block(0, ES_SIZE * index, ES_SIZE * index, ES_SIZE * (map->frame_num() - 1 - index)) = complement_infomat_block;
            complement_infomat.block(ES_SIZE * index, 0, ES_SIZE * (map->frame_num() - 1 - index), ES_SIZE * index) = complement_infomat_block.transpose();
        }

        pose_motion_infomat = complement_infomat;
        pose_motion_infovec = complement_infovec;
    }

    /* scope: create marginalization factor */ {
        Eigen::SelfAdjointEigenSolver<matrix<>> saesolver(pose_motion_infomat);

        vector<> lambdas = (saesolver.eigenvalues().array() > 1.0e-8).select(saesolver.eigenvalues(), 0);
        vector<> lambdas_inv = (saesolver.eigenvalues().array() > 1.0e-8).select(saesolver.eigenvalues().cwiseInverse(), 0);

        matrix<> sqrt_infomat = lambdas.cwiseSqrt().asDiagonal() * saesolver.eigenvectors().transpose();
        vector<> sqrt_infovec = lambdas_inv.cwiseSqrt().asDiagonal() * saesolver.eigenvectors().transpose() * pose_motion_infovec;

        std::vector<Frame *> remaining_frames;
        for (size_t i = 0; i < map->frame_num(); ++i) {
            if (i == index) continue;
            remaining_frames.emplace_back(map->get_frame(i));
        }
        map->set_marginalization_factor(Factor::create_marginalization_error(sqrt_infomat, sqrt_infovec, std::move(remaining_frames)));
    }
}

} // namespace pvio
