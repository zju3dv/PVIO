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
#include <pvio/core/feature_tracker.h>
#include <pvio/core/initializer.h>
#include <pvio/core/sliding_window_tracker.h>
#include <pvio/estimation/bundle_adjustor.h>
#include <pvio/estimation/pnp.h>
#include <pvio/geometry/essential.h>
#include <pvio/geometry/homography.h>
#include <pvio/geometry/lie_algebra.h>
#include <pvio/geometry/stereo.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/track.h>
#include <pvio/pvio.h>

namespace pvio {

Initializer::Initializer(std::shared_ptr<Config> config) :
    config(config) {
}

Initializer::~Initializer() = default;

void Initializer::mirror_keyframe_map(Map *feature_tracking_map, size_t init_frame_id) {
    size_t init_frame_index_last = feature_tracking_map->frame_index_by_id(init_frame_id);
    size_t init_frame_index_gap = config->initializer_keyframe_gap();
    size_t init_frame_index_distance = init_frame_index_gap * (config->sliding_window_size() - 1);

    init_frame_id = nil();

    if (init_frame_index_last < init_frame_index_distance) {
        map.reset();
        return;
    }

    size_t init_frame_index_first = init_frame_index_last - init_frame_index_distance;

    std::vector<size_t> init_keyframe_indices;
    for (size_t i = 0; i < config->sliding_window_size(); ++i) {
        init_keyframe_indices.push_back(init_frame_index_first + i * init_frame_index_gap);
    }

    map = std::make_unique<Map>();
    for (size_t index : init_keyframe_indices) {
        map->put_frame(feature_tracking_map->get_frame(index)->clone());
    }

    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *old_frame_i = feature_tracking_map->get_frame(init_keyframe_indices[j - 1]);
        Frame *old_frame_j = feature_tracking_map->get_frame(init_keyframe_indices[j]);
        Frame *new_frame_i = map->get_frame(j - 1);
        Frame *new_frame_j = map->get_frame(j);
        for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_frame_j); kj != nil()) {
                    new_frame_i->get_track(ki, create_if_empty)->add_keypoint(new_frame_j, kj);
                }
            }
        }
        new_frame_j->preintegration.data.clear();
        for (size_t f = init_keyframe_indices[j - 1]; f < init_keyframe_indices[j]; ++f) {
            Frame *old_frame = feature_tracking_map->get_frame(f + 1);
            std::vector<ImuData> &old_data = old_frame->preintegration.data;
            std::vector<ImuData> &new_data = new_frame_j->preintegration.data;
            new_data.insert(new_data.end(), old_data.begin(), old_data.end());
        }
    }
}

std::unique_ptr<SlidingWindowTracker> Initializer::initialize() {
    if (!map) return nullptr;
    if (!init_sfm()) return nullptr;
    if (!init_imu()) return nullptr;

    map->get_frame(0)->flag(FrameFlag::FF_FIX_POSE) = true;
    BundleAdjustor().solve(map.get(), config.get(), true);

    for (size_t i = 0; i < map->frame_num(); ++i) {
        map->get_frame(i)->flag(FrameFlag::FF_KEYFRAME) = true;
    }

    std::unique_ptr<SlidingWindowTracker> tracker = std::make_unique<SlidingWindowTracker>(std::move(map), config);
    return tracker;
}

bool Initializer::init_sfm() {
    // [1] try initializing using raw_map
    Frame *init_frame_i = map->get_frame(0);
    Frame *init_frame_j = map->get_frame(map->frame_num() - 1);

    double total_parallax = 0;
    int common_track_num = 0;
    std::vector<vector<3>> init_points;
    std::vector<std::pair<size_t, size_t>> init_matches;
    std::vector<char> init_point_status;
    std::vector<vector<2>> frame_i_keypoints;
    std::vector<vector<2>> frame_j_keypoints;
    matrix<3> init_R;
    vector<3> init_T;

    for (size_t ki = 0; ki < init_frame_i->keypoint_num(); ++ki) {
        Track *track = init_frame_i->get_track(ki);
        if (!track) continue;
        size_t kj = track->get_keypoint_index(init_frame_j);
        if (kj == nil()) continue;
        frame_i_keypoints.push_back(init_frame_i->get_keypoint(ki));
        frame_j_keypoints.push_back(init_frame_j->get_keypoint(kj));
        init_matches.emplace_back(ki, kj);
        total_parallax += (apply_k(init_frame_i->get_keypoint(ki), init_frame_i->K) - apply_k(init_frame_j->get_keypoint(kj), init_frame_j->K)).norm();
        common_track_num++;
    }

    if (common_track_num < (int)config->initializer_min_matches()) return false;
    total_parallax /= std::max(common_track_num, 1);
    if (total_parallax < config->initializer_min_parallax()) return false;

    std::vector<matrix<3>> Rs;
    std::vector<vector<3>> Ts;

    matrix<3> RH1, RH2;
    vector<3> TH1, TH2, nH1, nH2;
    matrix<3> H = find_homography_matrix(frame_i_keypoints, frame_j_keypoints, 0.7 / init_frame_i->K(0, 0), 0.999, 1000, config->random());
    if (!decompose_homography(H, RH1, RH2, TH1, TH2, nH1, nH2)) {
        log_message(PVIO_LOG_DEBUG, "SfM init fail: pure rotation.");
        return false; // is pure rotation
    }
    TH1 = TH1.normalized();
    TH2 = TH2.normalized();
    Rs.insert(Rs.end(), {RH1, RH1, RH2, RH2});
    Ts.insert(Ts.end(), {TH1, -TH1, TH2, -TH2});

    matrix<3> RE1, RE2;
    vector<3> TE;
    matrix<3> E = find_essential_matrix(frame_i_keypoints, frame_j_keypoints, 0.7 / init_frame_i->K(0, 0), 0.999, 1000, config->random());
    decompose_essential(E, RE1, RE2, TE);
    TE = TE.normalized();
    Rs.insert(Rs.end(), {RE1, RE1, RE2, RE2});
    Ts.insert(Ts.end(), {TE, -TE, TE, -TE});

    size_t triangulated_num = triangulate_from_rt_scored(frame_i_keypoints, frame_j_keypoints, Rs, Ts, config->initializer_min_triangulation(), init_points, init_R, init_T, init_point_status);

    if (triangulated_num < config->initializer_min_triangulation()) {
        log_message(PVIO_LOG_DEBUG, "SfM init fail: triangulation (%zd).", triangulated_num);
        return false;
    }

    // [2] create sfm map

    // [2.1] set init states
    PoseState pose;
    pose.q.setIdentity();
    pose.p.setZero();
    init_frame_i->set_pose(init_frame_i->camera, pose);
    pose.q = init_R.transpose();
    pose.p = -(init_R.transpose() * init_T);
    init_frame_j->set_pose(init_frame_j->camera, pose);

    for (size_t k = 0; k < init_points.size(); ++k) {
        if (init_point_status[k] == 0) continue;
        Track *track = init_frame_i->get_track(init_matches[k].first);
        track->set_landmark_point(init_points[k]);
        track->flag(TrackFlag::TF_VALID) = true;
    }

    // [2.2] solve other frames via pnp
    for (size_t j = 1; j + 1 < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        frame_j->set_pose(frame_j->camera, frame_i->get_pose(frame_i->camera));
        visual_inertial_pnp(map.get(), frame_j, config.get(), false);
    }

    // [2.3] triangulate more points
    for (size_t i = 0; i < map->track_num(); ++i) {
        Track *track = map->get_track(i);
        if (track->flag(TrackFlag::TF_VALID)) continue;
        track->triangulate();
    }

    // [3] sfm

    // [3.1] bundle adjustment
    map->get_frame(0)->flag(FrameFlag::FF_FIX_POSE) = true;
    if (!BundleAdjustor().solve(map.get(), config.get(), false)) {
        return false;
    }

    // [3.2] cleanup invalid points
    map->prune_tracks([](const Track *track) {
        return !track->flag(TrackFlag::TF_VALID) || track->landmark.quality > 1.0;
    });

    return true;
}

bool Initializer::init_imu() {
    reset_states();
    solve_gyro_bias();
    solve_gravity_scale_velocity();
    if (scale < 0.001 || scale > 1.0) return false;
    if (!config->initializer_refine_imu()) {
        return apply_init();
    }
    refine_scale_velocity_via_gravity();
    if (scale < 0.001 || scale > 1.0) return false;
    return apply_init();
}

void Initializer::solve_gyro_bias() {
    preintegrate();
    matrix<3> A = matrix<3>::Zero();
    vector<3> b = vector<3>::Zero();

    for (size_t j = 1; j < map->frame_num(); ++j) {
        const size_t i = j - 1;

        const Frame *frame_i = map->get_frame(i);
        const Frame *frame_j = map->get_frame(j);

        const PoseState pose_i = frame_i->get_pose(frame_i->imu);
        const PoseState pose_j = frame_j->get_pose(frame_j->imu);

        const quaternion &dq = frame_j->preintegration.delta.q;
        const matrix<3> &dq_dbg = frame_j->preintegration.jacobian.dq_dbg;
        A += dq_dbg.transpose() * dq_dbg;
        b += dq_dbg.transpose() * logmap((pose_i.q * dq).conjugate() * pose_j.q);
    }

    Eigen::JacobiSVD<matrix<3>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    bg = svd.solve(b);
}

void Initializer::solve_gravity_scale_velocity() {
    preintegrate();
    int N = (int)map->frame_num();
    matrix<> A;
    vector<> b;
    A.resize((N - 1) * 6, 3 + 1 + 3 * N);
    b.resize((N - 1) * 6);
    A.setZero();
    b.setZero();

    for (size_t j = 1; j < map->frame_num(); ++j) {
        const size_t i = j - 1;

        const Frame *frame_i = map->get_frame(i);
        const Frame *frame_j = map->get_frame(j);
        const PreIntegrator::Delta &delta = frame_j->preintegration.delta;
        const PoseState camera_pose_i = frame_i->get_pose(frame_i->camera);
        const PoseState camera_pose_j = frame_j->get_pose(frame_j->camera);

        A.block<3, 3>(i * 6, 0) = -0.5 * delta.t * delta.t * matrix<3>::Identity();
        A.block<3, 1>(i * 6, 3) = camera_pose_j.p - camera_pose_i.p;
        A.block<3, 3>(i * 6, 4 + i * 3) = -delta.t * matrix<3>::Identity();
        b.segment<3>(i * 6) = frame_i->pose.q * delta.p + (frame_j->pose.q * frame_j->camera.p_cs - frame_i->pose.q * frame_i->camera.p_cs);

        A.block<3, 3>(i * 6 + 3, 0) = -delta.t * matrix<3>::Identity();
        A.block<3, 3>(i * 6 + 3, 4 + i * 3) = -matrix<3>::Identity();
        A.block<3, 3>(i * 6 + 3, 4 + j * 3) = matrix<3>::Identity();
        b.segment<3>(i * 6 + 3) = frame_i->pose.q * delta.v;
    }

    vector<> x = A.fullPivHouseholderQr().solve(b);
    gravity = x.segment<3>(0).normalized() * PVIO_GRAVITY_NOMINAL;
    scale = x(3);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        velocities[i] = x.segment<3>(4 + i * 3);
    }
}

void Initializer::refine_scale_velocity_via_gravity() {
    static const double damp = 0.1;
    preintegrate();
    int N = (int)map->frame_num();
    matrix<> A;
    vector<> b;
    vector<> x;
    A.resize((N - 1) * 6, 2 + 1 + 3 * N);
    b.resize((N - 1) * 6);
    x.resize(2 + 1 + 3 * N);

    for (size_t iter = 0; iter < 1; ++iter) {
        A.setZero();
        b.setZero();
        matrix<3, 2> Tg = s2_tangential_basis(gravity);

        for (size_t j = 1; j < map->frame_num(); ++j) {
            const size_t i = j - 1;

            const Frame *frame_i = map->get_frame(i);
            const Frame *frame_j = map->get_frame(j);
            const PreIntegrator::Delta &delta = frame_j->preintegration.delta;
            const PoseState camera_pose_i = frame_i->get_pose(frame_i->camera);
            const PoseState camera_pose_j = frame_j->get_pose(frame_j->camera);

            A.block<3, 2>(i * 6, 0) = -0.5 * delta.t * delta.t * Tg;
            A.block<3, 1>(i * 6, 2) = camera_pose_j.p - camera_pose_i.p;
            A.block<3, 3>(i * 6, 3 + i * 3) = -delta.t * matrix<3>::Identity();
            b.segment<3>(i * 6) = 0.5 * delta.t * delta.t * gravity + frame_i->pose.q * delta.p + (frame_j->pose.q * frame_j->camera.p_cs - frame_i->pose.q * frame_i->camera.p_cs);

            A.block<3, 2>(i * 6 + 3, 0) = -delta.t * Tg;
            A.block<3, 3>(i * 6 + 3, 3 + i * 3) = -matrix<3>::Identity();
            A.block<3, 3>(i * 6 + 3, 3 + j * 3) = matrix<3>::Identity();
            b.segment<3>(i * 6 + 3) = delta.t * gravity + frame_i->pose.q * delta.v;
        }

        x = A.fullPivHouseholderQr().solve(b);
        vector<2> dg = x.segment<2>(0);
        gravity = (gravity + damp * Tg * dg).normalized() * PVIO_GRAVITY_NOMINAL;
    }

    scale = x(2);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        velocities[i] = x.segment<3>(3 + i * 3);
    }
}

void Initializer::reset_states() {
    bg.setZero();
    ba.setZero();
    gravity.setZero();
    scale = 1;
    velocities.resize(map->frame_num(), vector<3>::Zero());
}

void Initializer::preintegrate() {
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_j = map->get_frame(j);
        frame_j->preintegration.integrate(frame_j->image->t, bg, ba, true, false);
    }
}

bool Initializer::apply_init(bool apply_ba, bool apply_velocity) {
    static const vector<3> gravity_nominal{0, 0, -PVIO_GRAVITY_NOMINAL};

    quaternion q = quaternion::FromTwoVectors(gravity, gravity_nominal);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        PoseState imu_pose = frame->get_pose(frame->imu);
        imu_pose.q = q * imu_pose.q;
        imu_pose.p = scale * (q * imu_pose.p);
        frame->set_pose(frame->imu, imu_pose);
        if (apply_velocity) {
            frame->motion.v = q * velocities[i];
        } else {
            frame->motion.v.setZero();
        }
        frame->motion.bg = bg;
        if (apply_ba) {
            frame->motion.ba = ba;
        } else {
            frame->motion.ba.setZero();
        }
    }
    size_t result_point_num = 0;
    for (size_t i = 0; i < map->track_num(); ++i) {
        if (map->get_track(i)->triangulate()) {
            result_point_num++;
        }
    }

    return result_point_num >= config->initializer_min_landmarks();
}

} // namespace pvio
