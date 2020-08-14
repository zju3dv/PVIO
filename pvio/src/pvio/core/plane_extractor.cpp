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
#include <pvio/core/plane_extractor.h>
#include <pvio/forensics.h>
#include <pvio/geometry/stereo.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/plane.h>
#include <pvio/map/track.h>
#include <pvio/utility/ransac.h>
#include <pvio/utility/sector_area.h>

namespace pvio {

PlaneExtractor::PlaneExtractor(Map *map) :
    map(map) {
}

PlaneExtractor::~PlaneExtractor() = default;

bool PlaneExtractor::empty() const {
    return !extraction_issued;
}

void PlaneExtractor::work(std::unique_lock<std::mutex> &l) {
    std::vector<vector<3>> landmark_points;
    std::vector<size_t> track_ids;
    synchronized(map) {
        for (size_t i = 0; i < map->track_num(); ++i) {
            Track *track = map->get_track(i);
            if (track->flag(TrackFlag::TF_PLANE)) continue;
            if (track->flag(TrackFlag::TF_VALID) && enough_baseline(track) && track->life >= 10 && track->landmark.quality < 2.0) {
                landmark_points.emplace_back(track->get_landmark_point());
                track_ids.emplace_back(track->id());
            }
        }
    }
    extraction_issued = false;
    l.unlock();
    Ransac<3, PlaneState, PlaneSolver, PlaneEvaluator> ransac(0.03);
    PlaneRecord record;
    record.parameter = ransac.solve(landmark_points);
    if (ransac.inlier_count > 30) {
        vector<3> cog = vector<3>::Zero();
        for (size_t i = 0; i < landmark_points.size(); ++i) {
            if (ransac.inlier_mask[i]) {
                record.track_ids.emplace_back(track_ids[i]);
                cog += landmark_points[i];
            }
        }
        cog /= (double)ransac.inlier_count;
        matrix<3> cov = matrix<3>::Zero();
        for (size_t i = 0; i < landmark_points.size(); ++i) {
            if (ransac.inlier_mask[i]) {
                vector<3> dx = landmark_points[i] - cog;
                cov += dx * dx.transpose();
            }
        }
        Eigen::SelfAdjointEigenSolver<matrix<3>> saes(cov);
        record.parameter.normal = saes.eigenvectors().col(0);
        record.parameter.distance = record.parameter.normal.dot(cog);
        record.parameter.reference_point = cog;
        std::unique_lock planes_lock(planes_mutex);
        plane_records.emplace_back(std::move(record));
    }
}

void PlaneExtractor::update_map() {
    std::unique_lock planes_lock(planes_mutex);
    // if (map->plane_num() > 0) return;
    while (plane_records.size() > 0) {
        const PlaneRecord &record = plane_records.front();
        std::unique_ptr<Plane> plane = std::make_unique<Plane>();
        plane->parameter = record.parameter;
        for (size_t id : record.track_ids) {
            if (Track *track = map->get_track_by_id(id)) {
                track->flag(TrackFlag::TF_PLANE) = true;
                track->landmark.plane_id = plane->id();
                plane->tracks.insert(track);
            }
        }
        plane->update_sector_area();
        map->put_plane(std::move(plane));
        plane_records.pop_front();
    }
    for (size_t i = 0; i < map->plane_num(); ++i) {
        map->get_plane(i)->sector_area.centralize();
    }
}

void PlaneExtractor::issue_extraction() {
    auto l = lock();
    extraction_issued = true;
    resume(l);
}

void PlaneExtractor::extend_planes_and_cast_plane_points(double extend_rpe_ratio) {
    std::unique_lock planes_lock(planes_mutex);
    std::vector<Plane *> candidate_planes;
    std::vector<vector<3>> candidate_plane_points;
    for (size_t i = 0; i < map->track_num(); ++i) {
        Track *track = map->get_track(i);
        if (!track->flag(TrackFlag::TF_VALID)) continue; // it seems better to skip these invalid tracks
        vector<3> point = track->get_landmark_point();
        const auto &[frame_ref, keypoint_index_ref] = track->first_keypoint();
        const auto camera_ref_pose = frame_ref->get_pose(frame_ref->camera);
        double rpe_before_project = compute_reprojection_error(map, track, point);
        vector<3> direction = camera_ref_pose.q * frame_ref->get_keypoint(keypoint_index_ref).homogeneous();
        double min_rpe = std::numeric_limits<double>::max();
        vector<3> best_plane_point;
        size_t best_plane_id = nil();
        candidate_planes.clear();
        candidate_plane_points.clear();
        for (size_t j = 0; j < map->plane_num(); ++j) {
            Plane *plane = map->get_plane(j);
            if (plane->is_parallel(direction, 20)) continue;
            vector<3> plane_point = plane->cast_to_point(camera_ref_pose.p, direction);
            vector<3> y = camera_ref_pose.q.conjugate() * (plane_point - camera_ref_pose.p);
            if (y.z() < 0) continue;
            double rpe_after_project = compute_reprojection_error(map, track, plane_point);
            if (rpe_after_project < min_rpe) {
                min_rpe = rpe_after_project;
                best_plane_point = plane_point;
                best_plane_id = plane->id();
            }
            if (rpe_after_project / rpe_before_project < extend_rpe_ratio || rpe_after_project < 0.5) {
                candidate_planes.push_back(plane);
                candidate_plane_points.push_back(plane_point);
            }
        }

        for (size_t i = 0; i < candidate_planes.size(); ++i) {
            Plane *plane = candidate_planes[i];
            const vector<3> &plane_point = candidate_plane_points[i];
            if (plane->sector_area.is_near_boundary(plane_point, true, 1.2, 0.1)) {
                plane->tracks.insert(track);
                track->flag(TrackFlag::TF_PLANE) = true;
            }
        }

        if (track->flag(TrackFlag::TF_PLANE) && (best_plane_id != nil())) {
            track->set_landmark_point(best_plane_point);
            track->landmark.plane_id = best_plane_id;
        }
    }
}

void PlaneExtractor::merge_planes() {
    std::unique_lock planes_lock(planes_mutex);
    synchronized(map) {
        for (size_t i = 0; i < map->plane_num(); ++i) {
            for (size_t j = i + 1; j < map->plane_num(); ++j) {
                Plane *plane_i = map->get_plane(i);
                Plane *plane_j = map->get_plane(j);
                if (std::abs(plane_i->parameter.normal.dot(plane_j->parameter.normal)) < 0.95) continue;
                if (std::abs(plane_i->parameter.distance - plane_j->parameter.distance) > 0.25) continue;
                double overlap_ratio = plane_i->overlap_ratio(plane_j);
                if (overlap_ratio > 0.3) {
                    plane_i->merge(plane_j);
                    map->erase_plane(j);
                    i = 0;
                    break;
                }
            }
        }
    }
}

double PlaneExtractor::compute_reprojection_error(const Map *map, const Track *track, const vector<3> &point) {
    double rpe = 0;
    size_t rpe_num = 0;
    for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
        PoseState pose = frame->get_pose(frame->camera);
        vector<3> y = pose.q.conjugate() * (point - pose.p);
        rpe += (apply_k(y.hnormalized(), frame->K) - apply_k(frame->get_keypoint(keypoint_index), frame->K)).norm();
        rpe_num++;
    }
    if (rpe_num == 0) {
        return std::numeric_limits<double>::max();
    } else {
        return rpe / rpe_num;
    }
}

double PlaneExtractor::enough_baseline(const Track *track) {
    double baseline = track->compute_baseline();
    return (baseline > 0.5) || (track->landmark.inv_depth < (1 / 0.2) && baseline * track->landmark.inv_depth > 0.5);
}

} // namespace pvio
