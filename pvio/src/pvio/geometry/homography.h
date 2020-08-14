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
#ifndef PVIO_HOMOGRAPHY_H
#define PVIO_HOMOGRAPHY_H

#include <pvio/common.h>

namespace pvio {

bool decompose_homography(const matrix<3> &H, matrix<3> &R1, matrix<3> &R2, vector<3> &T1, vector<3> &T2, vector<3> &n1, vector<3> &n2);

// p2 = H * p1
matrix<3> solve_homography_4pt(const std::array<vector<2>, 4> &points1, const std::array<vector<2>, 4> &points2);

// d(p2, H * p1)
inline double homography_geometric_error(const matrix<3> &H, const vector<2> &p1, const vector<2> &p2) {
    return (p2 - (H * p1.homogeneous()).hnormalized()).squaredNorm();
}

} // namespace pvio

#endif // PVIO_HOMOGRAPHY_H
