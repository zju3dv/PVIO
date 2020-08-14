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
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/track.h>
#include <pvio/utility/poisson_disk_filter.h>

namespace pvio {

create_if_empty_t create_if_empty{};

struct Frame::construct_by_frame_t {};

Frame::Frame() :
    map(nullptr) {
}

Frame::Frame(size_t id, const construct_by_frame_t &construct_by_frame) :
    Identifiable(id), map(nullptr) {
}

Frame::~Frame() = default;

std::unique_ptr<Frame> Frame::clone() const {
    std::unique_ptr<Frame> frame = std::make_unique<Frame>(id(), construct_by_frame_t());
    frame->K = K;
    frame->sqrt_inv_cov = sqrt_inv_cov;
    frame->image = image;
    frame->pose = pose;
    frame->motion = motion;
    frame->camera = camera;
    frame->imu = imu;
    frame->preintegration = preintegration;
    frame->keypoints = keypoints;
    frame->tracks = std::vector<Track *>(keypoints.size(), nullptr);
    frame->reprojection_factors = std::vector<std::unique_ptr<Factor>>(keypoints.size());
    frame->map = nullptr;
    return frame;
}

void Frame::append_keypoint(const vector<2> &keypoint) {
    keypoints.emplace_back(keypoint);
    tracks.emplace_back(nullptr);
    reprojection_factors.emplace_back(nullptr);
}

Track *Frame::get_track(size_t keypoint_index, const create_if_empty_t &) {
    if (tracks[keypoint_index] == nullptr) {
        Track *track = map->create_track();
        track->add_keypoint(this, keypoint_index);
    }
    return tracks[keypoint_index];
}

void Frame::detect_keypoints(Config *config) {
    std::vector<vector<2>> pkeypoints(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
        pkeypoints[i] = apply_k(keypoints[i], K);
    }

    image->detect_keypoints(pkeypoints, config->feature_tracker_max_keypoint_detection(), config->feature_tracker_min_keypoint_distance());

    size_t old_keypoint_num = keypoints.size();
    keypoints.resize(pkeypoints.size());
    tracks.resize(pkeypoints.size(), nullptr);
    reprojection_factors.resize(pkeypoints.size());
    for (size_t i = old_keypoint_num; i < pkeypoints.size(); ++i) {
        keypoints[i] = remove_k(pkeypoints[i], K);
    }
}

void Frame::track_keypoints(Frame *next_frame, Config *config) {
    std::vector<vector<2>> curr_keypoints(keypoints.size());
    std::vector<vector<2>> next_keypoints;

    for (size_t i = 0; i < keypoints.size(); ++i) {
        curr_keypoints[i] = apply_k(keypoints[i], K);
    }

    if (config->feature_tracker_predict_keypoints()) {
        quaternion delta_key_q = (camera.q_cs.conjugate() * imu.q_cs * next_frame->preintegration.delta.q * next_frame->imu.q_cs.conjugate() * next_frame->camera.q_cs).conjugate();
        next_keypoints.resize(curr_keypoints.size());
        for (size_t i = 0; i < keypoints.size(); ++i) {
            next_keypoints[i] = apply_k((delta_key_q * keypoints[i].homogeneous()).hnormalized(), next_frame->K);
        }
    }

    std::vector<char> status;
    image->track_keypoints(next_frame->image.get(), curr_keypoints, next_keypoints, status);

    // filter keypoints based on track length
    std::vector<std::pair<size_t, size_t>> keypoint_index_track_length;
    keypoint_index_track_length.reserve(curr_keypoints.size());
    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        if (status[i] == 0) continue;
        Track *track = get_track(i);
        if (track == nullptr) continue;
        keypoint_index_track_length.emplace_back(i, track->keypoint_num());
    }

    std::sort(keypoint_index_track_length.begin(), keypoint_index_track_length.end(), [](const auto &a, const auto &b) {
        return a.second > b.second;
    });

    PoissonDiskFilter<2> filter(config->feature_tracker_min_keypoint_distance());
    for (auto &[keypoint_index, track_length] : keypoint_index_track_length) {
        vector<2> pt = next_keypoints[keypoint_index];
        if (filter.permit_point(pt)) {
            filter.preset_point(pt);
        } else {
            status[keypoint_index] = 0;
        }
    }

    for (size_t curr_keypoint_index = 0; curr_keypoint_index < curr_keypoints.size(); ++curr_keypoint_index) {
        if (status[curr_keypoint_index]) {
            size_t next_keypoint_index = next_frame->keypoint_num();
            next_frame->append_keypoint(remove_k(next_keypoints[curr_keypoint_index], next_frame->K));
            get_track(curr_keypoint_index, create_if_empty)->add_keypoint(next_frame, next_keypoint_index);
        }
    }
}

// void Frame::track_keypoints(Frame *next_frame, Config *config, std::function<bool(const Track *)> condition) {
//     std::vector<vector<2>> curr_keypoints;
//     std::vector<vector<2>> next_keypoints;
//     std::vector<size_t> curr_keypoints_ref;

//     for (size_t i = 0; i < keypoints.size(); ++i) {
//         if (tracks[i] && !condition(tracks[i])) continue;
//         curr_keypoints.emplace_back(apply_k(keypoints[i], K));
//         curr_keypoints_ref.emplace_back(i);
//     }

//     if (config->predict_keypoints()) {
//         quaternion delta_key_q = (camera.q_cs.conjugate() * imu.q_cs * next_frame->preintegration.delta.q * next_frame->imu.q_cs.conjugate() * next_frame->camera.q_cs).conjugate();
//         next_keypoints.resize(curr_keypoints.size());
//         for (size_t i = 0; i < curr_keypoints.size(); ++i) {
//             next_keypoints[i] = apply_k((delta_key_q * keypoints[curr_keypoints_ref[i]].homogeneous()).hnormalized(), next_frame->K);
//         }
//     }

//     std::vector<char> status;
//     image->track_keypoints(next_frame->image.get(), curr_keypoints, next_keypoints, status);

//     { // filter keypoints
//         std::vector<std::pair<size_t, size_t>> keypoint_ref_id_track_length;
//         keypoint_ref_id_track_length.reserve(curr_keypoints.size());
//         for (size_t i = 0; i < curr_keypoints.size(); ++i) {
//             if (status[i] == 0) continue;
//             Track *track = get_track(curr_keypoints_ref[i]);
//             if (track == nullptr) continue;
//             keypoint_ref_id_track_length.emplace_back(i, track->keypoint_num());
//         }
//         std::sort(keypoint_ref_id_track_length.begin(), keypoint_ref_id_track_length.end(), [](const auto &a, const auto &b) {
//             return a.second > b.second;
//         });
//         PoissonDiskFilter<2> filter(config->keypoint_distance());
//         for (auto &[keypoint_id, track_length] : keypoint_ref_id_track_length) {
//             vector<2> pt = next_keypoints[keypoint_id];
//             if (filter.permit_point(pt)) {
//                 filter.preset_point(pt);
//             } else {
//                 status[keypoint_id] = 0;
//             }
//         }
//     }

//     for (size_t ci = 0; ci < curr_keypoints.size(); ++ci) {
//         if (status[ci]) {
//             size_t next_keypoint_id = next_frame->keypoint_num();
//             next_frame->append_keypoint(remove_k(next_keypoints[ci], next_frame->K));
//             get_track(curr_keypoints_ref[ci], create_if_empty)->add_keypoint(next_frame, next_keypoint_id);
//         }
//     }
// }

PoseState Frame::get_pose(const ExtrinsicParams &sensor) const {
    PoseState result;
    result.q = pose.q * sensor.q_cs;
    result.p = pose.p + pose.q * sensor.p_cs;
    return result;
}

void Frame::set_pose(const ExtrinsicParams &sensor, const PoseState &pose) {
    this->pose.q = pose.q * sensor.q_cs.conjugate();
    this->pose.p = pose.p - this->pose.q * sensor.p_cs;
}

std::unique_lock<std::mutex> Frame::lock() const {
    if (map) {
        return map->lock();
    } else {
        return {};
    }
}

} // namespace pvio
