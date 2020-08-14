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
#include <pvio/geometry/essential.h>
#include <pvio/geometry/homography.h>
#include <pvio/geometry/lie_algebra.h>
#include <pvio/geometry/stereo.h>
#include <pvio/utility/ransac.h>

namespace pvio {

matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, double threshold, double confidence, size_t max_iteration, int seed) {
    std::vector<char> _;
    return find_essential_matrix(points1, points2, _, threshold, confidence, max_iteration, seed);
}

matrix<3> find_homography_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, double threshold, double confidence, size_t max_iteration, int seed) {
    std::vector<char> _;
    return find_homography_matrix(points1, points2, _, threshold, confidence, max_iteration, seed);
}

matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, std::vector<char> &inlier_mask, double threshold, double confidence, size_t max_iteration, int seed) {
    struct EssentialSolver {
        std::vector<matrix<3>> operator()(const std::array<vector<2>, 5> &samples1, const std::array<vector<2>, 5> &samples2) const {
            return solve_essential_5pt(samples1, samples2);
        }
    };
    struct EssentialEvaluator {
        const matrix<3> &E;
        const matrix<3> Et;
        EssentialEvaluator(const matrix<3> &E) :
            E(E), Et(E.transpose()) {
        }
        double operator()(const vector<2> &p1, const vector<2> &p2) const {
            return essential_geometric_error(E, p1, p2) + essential_geometric_error(Et, p2, p1);
        }
    };
    static const double t1 = 3.84;
    Ransac<5, matrix<3>, EssentialSolver, EssentialEvaluator> ransac(2.0 * t1 * threshold * threshold, confidence, max_iteration, seed);
    matrix<3> E = ransac.solve(points1, points2);
    inlier_mask.swap(ransac.inlier_mask);
    return E;
}

matrix<3> find_homography_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, std::vector<char> &inlier_mask, double threshold, double confidence, size_t max_iteration, int seed) {
    struct HomographySolver {
        matrix<3> operator()(const std::array<vector<2>, 4> &samples1, const std::array<vector<2>, 4> &samples2) const {
            return solve_homography_4pt(samples1, samples2);
        }
    };
    struct HomographyEvaluator {
        const matrix<3> &H;
        const matrix<3> Hinv;
        HomographyEvaluator(const matrix<3> &H) :
            H(H), Hinv(H.inverse()) {
        }
        double operator()(const vector<2> &p1, const vector<2> &p2) const {
            return homography_geometric_error(H, p1, p2) + homography_geometric_error(Hinv, p2, p1);
        }
    };
    static const double t2 = 5.99;
    Ransac<4, matrix<3>, HomographySolver, HomographyEvaluator> ransac(2.0 * t2 * threshold * threshold, confidence, max_iteration, seed);
    matrix<3> H = ransac.solve(points1, points2);
    inlier_mask.swap(ransac.inlier_mask);
    return H;
}

size_t triangulate_from_rt(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const matrix<3> &R, const vector<3> &T, std::vector<vector<3>> &result_points, std::vector<char> &result_status) {
    size_t count = 0;
    result_points.resize(points1.size());
    result_status.resize(points1.size());

    matrix<3, 4> P1, P2;
    P1.setIdentity();
    P2 << R, T;

    for (size_t i = 0; i < points1.size(); ++i) {
        if (triangulate_point_checked(P1, P2, points1[i], points2[i], result_points[i])) {
            result_status[i] = 1;
            count++;
        } else {
            result_status[i] = 0;
        }
    }
    return count;
}

size_t triangulate_from_rt(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const std::vector<matrix<3>> &Rs, const std::vector<vector<3>> &Ts, std::vector<vector<3>> &result_points, matrix<3> &result_R, vector<3> &result_T, std::vector<char> &result_status) {
    std::vector<std::vector<vector<3>>> points(Rs.size());
    std::vector<std::vector<char>> status(Rs.size());
    std::vector<size_t> counts(Rs.size());

    size_t best_i = 0;
    for (size_t i = 0; i < Rs.size(); ++i) {
        counts[i] = triangulate_from_rt(points1, points2, Rs[i], Ts[i], points[i], status[i]);
        if (counts[i] > counts[best_i]) {
            best_i = i;
        }
    }

    result_R = Rs[best_i];
    result_T = Ts[best_i];
    result_points.swap(points[best_i]);
    result_status.swap(status[best_i]);

    return counts[best_i];
}

size_t triangulate_from_rt_scored(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const matrix<3> &R, const vector<3> &T, std::vector<vector<3>> &result_points, std::vector<char> &result_status, double &score) {
    size_t count = 0;
    result_points.resize(points1.size());
    result_status.resize(points1.size());

    matrix<3, 4> P1, P2;
    P1.setIdentity();
    P2 << R, T;

    score = 0;

    for (size_t i = 0; i < points1.size(); ++i) {
        double current_score;
        if (triangulate_point_scored(P1, P2, points1[i], points2[i], result_points[i], current_score)) {
            result_status[i] = 1;
            score += current_score;
            count++;
        } else {
            result_status[i] = 0;
        }
    }

    score /= (double)std::max(count, size_t(1));
    return count;
}

size_t triangulate_from_rt_scored(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const std::vector<matrix<3>> &Rs, const std::vector<vector<3>> &Ts, size_t count_threshold, std::vector<vector<3>> &result_points, matrix<3> &result_R, vector<3> &result_T, std::vector<char> &result_status) {
    std::vector<std::vector<vector<3>>> points(Rs.size());
    std::vector<std::vector<char>> status(Rs.size());
    std::vector<size_t> counts(Rs.size());
    std::vector<double> scores(Rs.size());

    size_t best_i = 0;
    for (size_t i = 0; i < Rs.size(); ++i) {
        counts[i] = triangulate_from_rt_scored(points1, points2, Rs[i], Ts[i], points[i], status[i], scores[i]);
        if (counts[i] > count_threshold && scores[i] < scores[best_i]) {
            best_i = i;
        } else if (counts[i] > counts[best_i]) {
            best_i = i;
        }
    }

    result_R = Rs[best_i];
    result_T = Ts[best_i];
    result_points.swap(points[best_i]);
    result_status.swap(status[best_i]);

    return counts[best_i];
}

} // namespace pvio
