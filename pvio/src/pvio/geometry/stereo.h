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
#ifndef PVIO_STEREO_H
#define PVIO_STEREO_H

#include <pvio/common.h>

namespace pvio {

inline vector<2> apply_k(const vector<2> &p, const matrix<3> &K) {
    return {p(0) * K(0, 0) + K(0, 2), p(1) * K(1, 1) + K(1, 2)};
}

inline vector<2> remove_k(const vector<2> &p, const matrix<3> &K) {
    return {(p(0) - K(0, 2)) / K(0, 0), (p(1) - K(1, 2)) / K(1, 1)};
}

inline matrix<2, 3> dproj_dp(const vector<3> &p) {
    return (matrix<2, 3>() << 1.0 / p.z(), 0.0, -p.x() / (p.z() * p.z()),
            0.0, 1.0 / p.z(), -p.y() / (p.z() * p.z()))
        .finished();
}

matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, double threshold = 1.0, double confidence = 0.999, size_t max_iteration = 1000, int seed = 0);
matrix<3> find_homography_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, double threshold = 1.0, double confidence = 0.999, size_t max_iteration = 1000, int seed = 0);
matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, std::vector<char> &inlier_mask, double threshold = 1.0, double confidence = 0.999, size_t max_iteration = 1000, int seed = 0);
matrix<3> find_homography_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, std::vector<char> &inlier_mask, double threshold = 1.0, double confidence = 0.999, size_t max_iteration = 1000, int seed = 0);

vector<4> triangulate_point(const matrix<3, 4> &P1, const matrix<3, 4> &P2, const vector<2> &point1, const vector<2> &point2);
vector<4> triangulate_point(const std::vector<matrix<3, 4>> &Ps, const std::vector<vector<2>> &points);

bool triangulate_point_checked(const matrix<3, 4> &P1, const matrix<3, 4> &P2, const vector<2> &point1, const vector<2> &point2, vector<3> &p);
bool triangulate_point_checked(const std::vector<matrix<3, 4>> &Ps, const std::vector<vector<2>> &points, vector<3> &p);
bool triangulate_point_scored(const matrix<3, 4> &P1, const matrix<3, 4> &P2, const vector<2> &point1, const vector<2> &point2, vector<3> &p, double &score);
bool triangulate_point_scored(const std::vector<matrix<3, 4>> &Ps, const std::vector<vector<2>> &points, vector<3> &p, double &score);

size_t triangulate_from_rt(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const matrix<3> &R, const vector<3> &T, std::vector<vector<3>> &result_points, std::vector<char> &result_status);
size_t triangulate_from_rt(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const std::vector<matrix<3>> &Rs, const std::vector<vector<3>> &Ts, std::vector<vector<3>> &result_points, matrix<3> &result_R, vector<3> &result_T, std::vector<char> &result_status);

size_t triangulate_from_rt_scored(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const matrix<3> &R, const vector<3> &T, std::vector<vector<3>> &result_points, std::vector<char> &result_status, double &score);
size_t triangulate_from_rt_scored(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const std::vector<matrix<3>> &Rs, const std::vector<vector<3>> &Ts, size_t count_threshold, std::vector<vector<3>> &result_points, matrix<3> &result_R, vector<3> &result_T, std::vector<char> &result_status);

inline vector<4> triangulate_point(const matrix<3, 4> &P1, const matrix<3, 4> &P2, const vector<2> &point1, const vector<2> &point2) {
    matrix<4> A;
    A.row(0) = point1(0) * P1.row(2) - P1.row(0);
    A.row(1) = point1(1) * P1.row(2) - P1.row(1);
    A.row(2) = point2(0) * P2.row(2) - P2.row(0);
    A.row(3) = point2(1) * P2.row(2) - P2.row(1);
    return A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
}

inline vector<4> triangulate_point(const std::vector<matrix<3, 4>> &Ps, const std::vector<vector<2>> &points) {
    matrix<Eigen::Dynamic, 4> A(points.size() * 2, 4);
    for (size_t i = 0; i < points.size(); ++i) {
        A.row(i * 2 + 0) = points[i](0) * Ps[i].row(2) - Ps[i].row(0);
        A.row(i * 2 + 1) = points[i](1) * Ps[i].row(2) - Ps[i].row(1);
    }
    return A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
}

inline bool triangulate_point_checked(const matrix<3, 4> &P1, const matrix<3, 4> &P2, const vector<2> &point1, const vector<2> &point2, vector<3> &p) {
    double score;
    return triangulate_point_scored(P1, P2, point1, point2, p, score);
}

inline bool triangulate_point_checked(const std::vector<matrix<3, 4>> &Ps, const std::vector<vector<2>> &points, vector<3> &p) {
    double score;
    return triangulate_point_scored(Ps, points, p, score);
}

inline bool triangulate_point_scored(const matrix<3, 4> &P1, const matrix<3, 4> &P2, const vector<2> &point1, const vector<2> &point2, vector<3> &p, double &score) {
    vector<4> q = triangulate_point(P1, P2, point1, point2);
    score = 0;

    vector<3> q1 = P1 * q;
    vector<3> q2 = P2 * q;

    if (q1[2] * q[3] > 0 && q2[2] * q[3] > 0) {
        if (q1[2] / q[3] < 100 && q2[2] / q[3] < 100) {
            p = q.hnormalized();
            score = 0.5 * ((q1.hnormalized() - point1).squaredNorm() + (q2.hnormalized() - point2).squaredNorm());
            return true;
        }
    }

    return false;
}

inline bool triangulate_point_scored(const std::vector<matrix<3, 4>> &Ps, const std::vector<vector<2>> &points, vector<3> &p, double &score) {
    //if (Ps.size() < 2) return false;
    runtime_assert(Ps.size() >=2 , "Ps.size() <2 in triangulate_point_scored.");
    bool has_parallax = true;
    vector<4> q = triangulate_point(Ps, points);
    score = 0;

    for (size_t i = 0; i < points.size(); ++i) {
        vector<3> qi = Ps[i] * q;
        if (!(qi[2] * q[3] > 0)) {
            has_parallax = false;
        }
        if (!(qi[2] / q[3] < 100)) {
            has_parallax = false;
        }
        score += (qi.hnormalized() - points[i]).squaredNorm();
    }
    score /= points.size();
    if(has_parallax) {
        p = q.hnormalized();
    } else {
        p = q.segment<3>(0).normalized();
    }
    return has_parallax;
}

} // namespace pvio

#endif // PVIO_STEREO_H
