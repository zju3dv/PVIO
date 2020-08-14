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
#include <pvio/map/plane.h>
#include <pvio/utility/ransac.h>

namespace pvio {

void Plane::merge(Plane *plane) {
    for (Track *track : plane->tracks) {
        track->landmark.plane_id = id();
        tracks.insert(track);
    }
    sector_area.merge(plane->sector_area);
    update_parameter(true);
    parameter.reference_point = point_project_to_plane(sector_area.center());
    sector_area.update_parameter(parameter.reference_point, parameter.normal, true);
}

double Plane::overlap_ratio(Plane *plane) const {
    const Plane *pa;
    const Plane *pb;
    if (plane->tracks.size() < tracks.size()) {
        pa = plane;
        pb = this;
    } else {
        pa = this;
        pb = plane;
    }
    if (pa->tracks.size() == 0) return 0;

    double count = 0;
    for (Track *track : pa->tracks) {
        if (pb->tracks.count(track)) {
            count++;
        }
    }
    return count / (double)pa->tracks.size();
}

void Plane::update_sector_area() {
    sector_area.update_parameter(parameter.reference_point, parameter.normal);
    for (const auto &track : tracks) {
        vector<3> plane_point = point_project_to_plane(track->get_landmark_point());
        sector_area.insert_point(plane_point);
    }
}

void Plane::update_parameter(bool use_ransac) {
    std::vector<vector<3>> landmark_points;
    landmark_points.reserve(tracks.size());
    for (const auto &track : tracks) {
        if (!track->flag(TrackFlag::TF_VALID) || !PlaneExtractor::enough_baseline(track) || track->life < 15) continue;
        vector<3> p;
        // p = track->get_landmark_point();
        track->try_triangulate(p);
        landmark_points.push_back(p);
    }
    if (landmark_points.size() < 50) return;
    if (use_ransac) {
        Ransac<3, PlaneState, PlaneExtractor::PlaneSolver, PlaneExtractor::PlaneEvaluator> ransac(0.05);
        parameter = ransac.solve(landmark_points);
        if (ransac.inlier_count > 30) {
            vector<3> cog = vector<3>::Zero();
            for (size_t i = 0; i < landmark_points.size(); ++i) {
                if (ransac.inlier_mask[i]) {
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
            parameter.normal = saes.eigenvectors().col(0);
            parameter.distance = parameter.normal.dot(cog);
            parameter.reference_point = cog;
        }
    } else {
        vector<3> cog = vector<3>::Zero();
        for (size_t i = 0; i < landmark_points.size(); ++i) {
            cog += landmark_points[i];
        }
        cog /= double(landmark_points.size());
        matrix<3> cov = matrix<3>::Zero();
        for (size_t i = 0; i < landmark_points.size(); ++i) {
            vector<3> dx = landmark_points[i] - cog;
            cov += dx * dx.transpose();
        }
        Eigen::SelfAdjointEigenSolver<matrix<3>> saes(cov);
        parameter.normal = saes.eigenvectors().col(0);
        parameter.distance = parameter.normal.dot(cog);
        parameter.reference_point = cog;
    }
}

double Plane::point_to_plane_abs_distance(const vector<3> &point) const {
    return std::abs(parameter.normal.dot(point) - parameter.distance);
}

double Plane::cast_to_depth(const vector<3> &origin, const vector<3> &direction) const {
    return (parameter.distance - parameter.normal.dot(origin)) / parameter.normal.dot(direction);
}

vector<3> Plane::cast_to_point(const vector<3> &origin, const vector<3> &direction) const {
    return origin + (direction * cast_to_depth(origin, direction));
}

vector<3> Plane::point_project_to_plane(const vector<3> &point) const {
    return point - (parameter.normal.dot(point) - parameter.distance) * parameter.normal;
}

bool Plane::is_parallel(const vector<3> &direction, double angle) const {
    return std::abs(direction.normalized().dot(parameter.normal)) < std::sin(angle * (M_PI / 180));
}

} // namespace pvio
