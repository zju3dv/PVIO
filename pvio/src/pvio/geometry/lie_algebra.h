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
#ifndef PVIO_LIE_ALGEBRA_H
#define PVIO_LIE_ALGEBRA_H

#include <pvio/common.h>

namespace pvio {

inline matrix<3> hat(const vector<3> &w) {
    return (matrix<3>() << 0, -w.z(), w.y(),
            w.z(), 0, -w.x(),
            -w.y(), w.x(), 0)
        .finished();
}

inline quaternion expmap(const vector<3> &w) {
    Eigen::AngleAxisd aa(w.norm(), w.stableNormalized());
    quaternion q;
    q = aa;
    return q;
}

inline vector<3> logmap(const quaternion &q) {
    Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
}

matrix<3> right_jacobian(const vector<3> &w);
matrix<3, 2> s2_tangential_basis(const vector<3> &x);
matrix<3, 2> s2_tangential_basis_barrel(const vector<3> &x);

} // namespace pvio

#endif // PVIO_LIE_ALGEBRA_H
