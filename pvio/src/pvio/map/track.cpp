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
#include <pvio/estimation/factor.h>
#include <pvio/geometry/stereo.h>
#include <pvio/map/track.h>

namespace pvio {

Track::Track() :
    life(0){};
Track::~Track() = default;

const vector<2> &Track::get_keypoint(Frame *frame) const {
    return frame->get_keypoint(keypoint_refs.at(frame));
}

void Track::add_keypoint(Frame *frame, size_t keypoint_index) {
    frame->tracks[keypoint_index] = this;
    frame->reprojection_factors[keypoint_index] = Factor::create_reprojection_error(this, frame, keypoint_index);
    keypoint_refs[frame] = keypoint_index;
    life++;
}

void Track::remove_keypoint(Frame *frame, bool suicide_if_empty) {
    size_t keypoint_index = keypoint_refs.at(frame);
    if (keypoint_refs.size() > 1) {
        if (frame == first_frame()) {
            const auto [next_frame, next_keypoint_index] = *std::next(keypoint_refs.begin());
            auto curr_camera = frame->get_pose(frame->camera);
            vector<3> point = (curr_camera.q * frame->get_keypoint(keypoint_index).homogeneous()) / landmark.inv_depth + curr_camera.p;
            auto next_camera = next_frame->get_pose(next_frame->camera);
            double next_depth = 1.0 / (next_camera.q.conjugate() * (point - next_camera.p)).z();
            landmark.inv_depth = next_depth;
        }
    } else {
        flag(TrackFlag::TF_VALID) = false;
    }
    frame->tracks[keypoint_index] = nullptr;
    frame->reprojection_factors[keypoint_index].reset();
    keypoint_refs.erase(frame);
    if (suicide_if_empty && keypoint_refs.size() == 0) {
        map->recycle_track(this);
    }
}

bool Track::try_triangulate(vector<3> &p) {
    bool valid = false;
    std::vector<matrix<3, 4>> Ps;
    std::vector<vector<2>> ps;
    for (const auto &[frame, keypoint_index] : keypoint_map()) {
        matrix<3, 4> P;
        matrix<3, 3> R;
        vector<3> T;
        auto pose = frame->get_pose(frame->camera);
        R = pose.q.conjugate().matrix();
        T = -(R * pose.p);
        P << R, T;
        Ps.push_back(P);
        ps.push_back(frame->get_keypoint(keypoint_index));
    }
    if (triangulate_point_checked(Ps, ps, p)) {
        return true;
    } else {
        return false;
    }
}

bool Track::triangulate() {
    std::vector<matrix<3, 4>> Ps;
    std::vector<vector<2>> ps;
    for (const auto &[frame, keypoint_index] : keypoint_map()) {
        matrix<3, 4> P;
        matrix<3, 3> R;
        vector<3> T;
        auto pose = frame->get_pose(frame->camera);
        R = pose.q.conjugate().matrix();
        T = -(R * pose.p);
        P << R, T;
        Ps.push_back(P);
        ps.push_back(frame->get_keypoint(keypoint_index));
    }
    vector<3> p;
    if (triangulate_point_checked(Ps, ps, p)) {
        set_landmark_point(p);
        flag(TrackFlag::TF_VALID) = true;
    } else {
        flag(TrackFlag::TF_VALID) = false;
    }
    flag(TrackFlag::TF_TRIANGULATED) = true;
    return flag(TrackFlag::TF_VALID);
}

double Track::compute_parallax() const {
    double total_parallax = 0;
    for (auto it_i = keypoint_refs.begin();; ++it_i) {
        auto it_j = std::next(it_i);
        if (it_j == keypoint_refs.end()) break;
        const Frame *frame_i = it_i->first;
        const Frame *frame_j = it_j->first;
        const size_t ki = it_i->second;
        const size_t kj = it_j->second;
        quaternion qij = (frame_i->camera.q_cs.conjugate() * frame_i->imu.q_cs * frame_j->preintegration.delta.q * frame_j->imu.q_cs.conjugate() * frame_j->camera.q_cs).conjugate();
        vector<2> pi = apply_k((qij * frame_i->get_keypoint(ki).homogeneous()).hnormalized(), frame_i->K);
        vector<2> pj = apply_k(frame_j->get_keypoint(kj), frame_j->K);
        total_parallax += (pi - pj).norm();
    }
    return total_parallax;
}

double Track::compute_baseline() const {
    double total_baseline = 0;
    for (auto it_i = keypoint_refs.begin();; ++it_i) {
        auto it_j = std::next(it_i);
        if (it_j == keypoint_refs.end()) break;
        const Frame *frame_i = it_i->first;
        const Frame *frame_j = it_j->first;
        total_baseline += (frame_i->pose.p - frame_j->pose.p).norm();
    }
    return total_baseline;
}

vector<3> Track::get_landmark_point() const {
    const auto &[frame, keypoint_index] = first_keypoint();
    auto camera = frame->get_pose(frame->camera);
    return camera.q * frame->get_keypoint(keypoint_index).homogeneous() / landmark.inv_depth + camera.p;
}

void Track::set_landmark_point(const vector<3> &p) {
    const auto [frame, keypoint_index] = first_keypoint();
    auto camera = frame->get_pose(frame->camera);
    landmark.inv_depth = 1.0 / (camera.q.conjugate() * (p - camera.p)).z();
}

} // namespace pvio
