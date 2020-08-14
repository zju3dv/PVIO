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
#ifndef PVIO_QUATERNION_PARAMETERIZATION_H
#define PVIO_QUATERNION_PARAMETERIZATION_H

#include <ceres/ceres.h>
#include <pvio/common.h>
#include <pvio/geometry/lie_algebra.h>

namespace pvio {

struct QuaternionParameterization : public ceres::LocalParameterization {
    bool Plus(const double *q, const double *dq, double *q_plus_dq) const override {
        map<quaternion> result(q_plus_dq);
        result = (const_map<quaternion>(q) * expmap(const_map<vector<3>>(dq))).normalized();
        return true;
    }
    bool ComputeJacobian(const double *, double *jacobian) const override {
        map<matrix<4, 3, true>> J(jacobian);
        J.setIdentity(); // the composited jacobian is computed in PreIntegrationError::Evaluate(), we simply forward it.
        return true;
    }
    int GlobalSize() const override {
        return 4;
    }
    int LocalSize() const override {
        return 3;
    }
};

} // namespace pvio

#endif // PVIO_QUATERNION_PARAMETERIZATION_H
