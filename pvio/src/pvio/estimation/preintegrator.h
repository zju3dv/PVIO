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
#ifndef PVIO_PREINTEGRATOR_H
#define PVIO_PREINTEGRATOR_H

#include <pvio/common.h>
#include <pvio/estimation/state.h>

namespace pvio {

class Frame;

struct PreIntegrator {
    struct Delta {
        double t;
        quaternion q;
        vector<3> p;
        vector<3> v;
        matrix<15> cov; // ordered in q, p, v, bg, ba
        matrix<15> sqrt_inv_cov;
    };

    struct Jacobian {
        matrix<3> dq_dbg;
        matrix<3> dp_dbg;
        matrix<3> dp_dba;
        matrix<3> dv_dbg;
        matrix<3> dv_dba;
    };

    void reset();
    void increment(double dt, const ImuData &data, const vector<3> &bg, const vector<3> &ba, bool compute_jacobian, bool compute_covariance);
    bool integrate(double t, const vector<3> &bg, const vector<3> &ba, bool compute_jacobian, bool compute_covariance);
    void compute_sqrt_inv_cov();

    void predict(const Frame *old_frame, Frame *new_frame);

    matrix<3> cov_w; // continuous noise covariance
    matrix<3> cov_a;
    matrix<3> cov_bg; // continuous random walk noise covariance
    matrix<3> cov_ba;

    Delta delta;
    Jacobian jacobian;

    std::vector<ImuData> data;
};

} // namespace pvio

#endif // PVIO_PREINTEGRATOR_H
