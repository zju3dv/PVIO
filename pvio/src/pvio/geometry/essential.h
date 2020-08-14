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
#ifndef PVIO_ESSENTIAL_H
#define PVIO_ESSENTIAL_H

#include <pvio/common.h>

namespace pvio {

void decompose_essential(const matrix<3> &E, matrix<3> &R1, matrix<3> &R2, vector<3> &T);

std::vector<matrix<3>> solve_essential_5pt(const std::array<vector<2>, 5> &points1, const std::array<vector<2>, 5> &points2);

inline double essential_geometric_error(const matrix<3> &E, const vector<2> &p1, const vector<2> &p2) {
    vector<3> Ep1 = E * p1.homogeneous();
    double r = p2.homogeneous().transpose() * Ep1;
    return r * r / Ep1.segment<2>(0).squaredNorm();
}

} // namespace pvio

#endif // PVIO_ESSENTIAL_H
