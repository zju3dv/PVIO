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
#ifndef PVIO_WAHBA_PROBLEM_H
#define PVIO_WAHBA_PROBLEM_H

#include <Eigen/Eigen>
#include <pvio/common.h>
#include <vector>

namespace pvio {

inline matrix<3> kabsch(const std::vector<vector<3>> &src, const std::vector<vector<3>> &dst) {
    matrix<3> cov = matrix<3>::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        cov += src[i] * dst[i].transpose();
    }
    cov = cov * (1.0 / src.size());
    Eigen::JacobiSVD<matrix<3>> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const matrix<3> &U = svd.matrixU();
    const matrix<3> &V = svd.matrixV();
    matrix<3> E = matrix<3>::Identity();
    if ((V * U.transpose()).determinant() >= 0.0) {
        E(2, 2) = 1.0;
    } else {
        E(2, 2) = -1.0;
    }

    return V * E * U.transpose();
}

inline std::tuple<double, matrix<3>, vector<3>> find_srt(std::vector<vector<3>> src, std::vector<vector<3>> dst) {
    vector<3> src_avg = vector<3>::Zero();
    vector<3> dst_avg = vector<3>::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        src_avg += src[i];
        dst_avg += dst[i];
    }
    src_avg /= (double)src.size();
    dst_avg /= (double)dst.size();

    double src_d2 = 0;
    double dst_d2 = 0;
    for (size_t i = 0; i < src.size(); ++i) {
        src[i] -= src_avg;
        dst[i] -= dst_avg;
        src_d2 += src[i].squaredNorm();
        dst_d2 += dst[i].squaredNorm();
    }

    double S = sqrt(dst_d2 / src_d2);
    matrix<3> R = kabsch(src, dst);
    vector<3> T = dst_avg - S * R * src_avg;

    return {S, R, T};
}

} // namespace pvio

#endif
