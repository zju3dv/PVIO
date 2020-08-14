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
#include <pvio/common.h>
#include <pvio/core/feature_tracker.h>
#include <pvio/core/frontend_worker.h>
#include <pvio/core/plane_extractor.h>
#include <pvio/core/sliding_window_tracker.h>
#include <pvio/estimation/bundle_adjustor.h>
#include <pvio/estimation/factor.h>
#include <pvio/estimation/pnp.h>
#include <pvio/forensics.h>
#include <pvio/geometry/lie_algebra.h>
#include <pvio/geometry/stereo.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/track.h>
#include <pvio/map/plane.h>
#include <pvio/utility/unique_timer.h>

namespace pvio {

SlidingWindowTracker::SlidingWindowTracker(std::unique_ptr<Map> keyframe_map, std::shared_ptr<Config> config) :
    map(std::move(keyframe_map)), config(config), skipped_frames(0) {
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        frame_j->preintegration.integrate(frame_j->image->t, frame_i->motion.bg, frame_i->motion.ba, true, true);
    }
    plane_extractor = std::make_unique<PlaneExtractor>(map.get());
    plane_extractor->start();
}

SlidingWindowTracker::~SlidingWindowTracker() {
    plane_extractor->stop();
}

void SlidingWindowTracker::mirror_frame(Map *feature_tracking_map, size_t frame_id) {
    Frame *new_frame_i = map->last_frame();

    size_t frame_index_i = feature_tracking_map->frame_index_by_id(new_frame_i->id());
    size_t frame_index_j = feature_tracking_map->frame_index_by_id(frame_id);

    if (frame_index_i == nil() || frame_index_j == nil()) return;

    Frame *old_frame_i = feature_tracking_map->get_frame(frame_index_i);
    Frame *old_frame_j = feature_tracking_map->get_frame(frame_index_j);

    frame = old_frame_j->clone();
    Frame *new_frame_j = frame.get();

    for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
        if (Track *track = old_frame_i->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(old_frame_j); kj != nil()) {
                new_frame_i->get_track(ki, create_if_empty)->add_keypoint(new_frame_j, kj);
            }
        }
    }
}

bool SlidingWindowTracker::track() {
    Frame *last_frame = map->last_frame();
    frame->preintegration.integrate(frame->image->t, last_frame->motion.bg, last_frame->motion.ba, true, true);
    frame->preintegration.predict(last_frame, frame.get());
    visual_inertial_pnp(map.get(), frame.get(), config.get(), true);

    keyframe_check(frame.get());

    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        Track *track = frame->get_track(i);
        if (!track) continue;
        if (track->flag(TrackFlag::TF_VALID)) continue;
        track->triangulate();
    }

    if (last_frame->flag(FrameFlag::FF_KEYFRAME)) {
        while (map->frame_num() >= config->sliding_window_size() + 1) {
            map->marginalize_frame(0);
        }
        map->put_frame(std::move(frame));

#ifdef PVIO_ENABLE_PLANE_CONSTRAINT
        plane_extractor->update_map();
        plane_extractor->extend_planes_and_cast_plane_points(1.2);
#endif
        if (!map->get_marginalization_factor()) {
            std::vector<Frame *> init_frames;
            for (size_t i = 1; i < map->frame_num(); ++i) {
                init_frames.push_back(map->get_frame(i - 1));
            }
            matrix<> init_infomat(ES_SIZE * (map->frame_num() - 1), ES_SIZE * (map->frame_num() - 1));
            vector<> init_infovec(ES_SIZE * (map->frame_num() - 1));
            init_infomat.setZero();
            init_infovec.setZero();
            init_infomat.block<3, 3>(ES_P, ES_P) = 1.0e15 * matrix<3>::Identity();
            init_infomat.block<3, 3>(ES_Q, ES_Q) = 1.0e15 * matrix<3>::Identity();
            map->set_marginalization_factor(Factor::create_marginalization_error(init_infomat, init_infovec, std::move(init_frames)));
        }
        BundleAdjustor().solve(map.get(), config.get(), true);

    } else {
        const std::vector<ImuData> &data = last_frame->preintegration.data;
        frame->preintegration.data.insert(frame->preintegration.data.begin(), data.begin(), data.end());
        frame->preintegration.integrate(frame->image->t, last_frame->motion.bg, last_frame->motion.ba, true, true);
        map->erase_frame(map->frame_num() - 1);
        map->put_frame(std::move(frame));
    }

    map->prune_tracks([](const Track *track) {
        return (!track->flag(TrackFlag::TF_VALID) || track->landmark.quality > 3.0) && (!track->flag(TrackFlag::TF_PLANE) || track->landmark.quality > 3.0);
    });

    if (last_frame->flag(FrameFlag::FF_KEYFRAME)) {
        plane_extractor->merge_planes();
        for (size_t i = 0; i < map->plane_num(); ++i) {
            Plane *plane = map->get_plane(i);
            plane->update_parameter(true);
            plane->sector_area.centralize();
        }
    }

    plane_extractor->issue_extraction();

    forensics(sliding_window_track_painter, p) {
        if (p.has_value()) {
            auto painter = std::any_cast<ForensicsPainter *>(p);
            auto frame = map->last_frame();
            painter->set_image(frame->image.get());
            for (size_t i = 0; i < frame->keypoint_num(); ++i) {
                if (Track *track = frame->get_track(i)) {
                    size_t h = track->id() * 6364136223846793005u + 1442695040888963407;
                    color3b c = {h & 0xFF, (h >> 4) & 0xFF, (h >> 8) & 0xFF};
                    painter->point(apply_k(frame->get_keypoint(i), frame->K).cast<int>(), c, 5);
                    auto it_j = track->keypoint_map().rbegin();
                    auto it_i = std::next(it_j);
                    for (; it_i != track->keypoint_map().rend(); ++it_i, ++it_j) {
                        vector<2> pi = apply_k(track->get_keypoint(it_i->first), it_i->first->K);
                        vector<2> pj = apply_k(track->get_keypoint(it_j->first), it_j->first->K);
                        if (it_i->first->flag(FrameFlag::FF_KEYFRAME)) {
                            painter->point(pi.cast<int>(), c, 3);
                        }
                        painter->line(pj.cast<int>(), pi.cast<int>(), c, 2);
                    }
                }
            }
        }
    }

    forensics(sliding_window_reprojection_painter, p) {
        if (p.has_value()) {
            ForensicsPainter *painter = std::any_cast<ForensicsPainter *>(p);
            Frame *frame = map->last_frame();
            painter->set_image(frame->image.get());

            auto camera_pose = frame->get_pose(frame->camera);

            for (size_t i = 0; i < frame->keypoint_num(); ++i) {
                if (Track *track = frame->get_track(i)) {
                    if (!track->flag(TrackFlag::TF_VALID)) continue;
                    vector<2> keypoint = apply_k(frame->get_keypoint(i), frame->K);
                    vector<2> reprojection = apply_k((camera_pose.q.conjugate() * (track->get_landmark_point() - camera_pose.p)).hnormalized(), frame->K);
                    painter->line(keypoint.cast<int>(), reprojection.cast<int>(), {255, 0, 0}, 2);
                    painter->point(keypoint.cast<int>(), {255, 0, 255}, 5);
                    painter->point(reprojection.cast<int>(), {64, 64, 255}, 5);
                }
            }
        }
    }

    forensics(sliding_window_landmarks, landmarks) {
        std::vector<OutputMapPoint> points;
        points.reserve(map->track_num());
        for (size_t i = 0; i < map->track_num(); ++i) {
            if (Track *track = map->get_track(i)) {
                if (track->flag(TrackFlag::TF_VALID)) {
                    OutputMapPoint point;
                    point.p = track->get_landmark_point();
                    if (track->flag(TrackFlag::TF_PLANE)) {
                        for (size_t j = 0; j < map->plane_num(); ++j) {
                            point.reserved = track->landmark.plane_id + 1;
                        }
                    } else {
                        point.reserved = 0;
                    }
                    points.push_back(point);
                }
            }
        }
        landmarks = std::move(points);
    }

    forensics(sliding_window_planes, map_planes) {
        std::vector<OutputPlane> planes;
        planes.reserve(map->plane_num());
        for (size_t i = 0; i < map->plane_num(); ++i) {
            if (Plane *plane = map->get_plane(i)) {
                OutputPlane op;
                op.normal = plane->parameter.normal;
                op.distance = plane->parameter.distance;
                op.id = plane->id();
                op.reference_point = plane->parameter.reference_point;
                op.track_ids.reserve(plane->tracks.size());
                for (const auto &track : plane->tracks) {
                    op.track_ids.push_back(track->id());
                }
                op.vertices.reserve(plane->sector_area.sectors().size());
                for (const auto &v : plane->sector_area.sectors()) {
                    if (v.first > 0) op.vertices.emplace_back(v.second);
                }
                planes.emplace_back(std::move(op));
            }
        }
        map_planes = std::move(planes);
    }

    forensics(sliding_window_keyframe_poses, keyframe_poses) {
        std::vector<OutputState> poses;
        for (size_t i = 0; i < map->frame_num(); ++i) {
            Frame *frame = map->get_frame(i);
            auto camera_pose = frame->get_pose(frame->camera);
            OutputState pose;
            pose.t = frame->image->t;
            pose.p = camera_pose.p;
            pose.q = camera_pose.q;
            pose.v = frame->motion.v;
            pose.ba = frame->motion.ba;
            pose.bg = frame->motion.bg;
            poses.push_back(pose);
        }
        keyframe_poses = std::move(poses);
    }

    return true;
}

std::tuple<PoseState, MotionState> SlidingWindowTracker::get_latest_state() const {
    const Frame *frame = map->last_frame();
    return {frame->pose, frame->motion};
}

void SlidingWindowTracker::keyframe_check(Frame *frame) {
    Frame *frame_i = nullptr;
    for (size_t i = 0; i < map->frame_num(); ++i) {
        if (map->get_frame(map->frame_num() - i - 1)->flag(FrameFlag::FF_KEYFRAME)) {
            frame_i = map->get_frame(map->frame_num() - i - 1);
            break;
        }
    }
    if (!frame_i) {
        frame->flag(FrameFlag::FF_KEYFRAME) = true;
    } else {
        Frame *frame_j = frame;
        quaternion qij = (frame_i->camera.q_cs.conjugate() * frame_i->imu.q_cs * frame_j->preintegration.delta.q * frame_j->imu.q_cs.conjugate() * frame_j->camera.q_cs).conjugate();
        std::vector<double> parallax_list;
        for (size_t kj = 0; kj < frame_j->keypoint_num(); ++kj) {
            Track *track = frame_j->get_track(kj);
            if (!track) continue;
            size_t ki = track->get_keypoint_index(frame_i);
            if (ki == nil()) continue;
            vector<2> pi = apply_k((qij * frame_i->get_keypoint(ki).homogeneous()).hnormalized(), frame_i->K);
            vector<2> pj = apply_k(frame_j->get_keypoint(kj), frame_j->K);
            parallax_list.push_back((pi - pj).norm());
        }
        if (parallax_list.size() < 50) {
            frame->flag(FrameFlag::FF_KEYFRAME) = true;
        } else {
            std::sort(parallax_list.begin(), parallax_list.end());
            double parallax = parallax_list[parallax_list.size() * 4 / 5];
            if (parallax > 50) {
                frame->flag(FrameFlag::FF_KEYFRAME) = true;
            } else {
                skipped_frames++;
            }
        }
    }
    if (skipped_frames > 10) {
        frame->flag(FrameFlag::FF_KEYFRAME) = true;
    }
    if (frame->flag(FrameFlag::FF_KEYFRAME)) {
        skipped_frames = 0;
    }
}

} // namespace pvio
