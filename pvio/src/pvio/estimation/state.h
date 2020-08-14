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
#ifndef PVIO_STATE_H
#define PVIO_STATE_H

#include <pvio/common.h>

namespace pvio {

class Frame;
class Track;
class Map;

enum ErrorStateLocation {
    ES_Q = 0,
    ES_P = 3,
    ES_V = 6,
    ES_BG = 9,
    ES_BA = 12,
    ES_SIZE = 15
};

struct ExtrinsicParams {
    quaternion q_cs;
    vector<3> p_cs;
};

struct PoseState {
    PoseState() {
        q.setIdentity();
        p.setZero();
    }

    quaternion q;
    vector<3> p;
};

struct MotionState {
    MotionState() {
        v.setZero();
        bg.setZero();
        ba.setZero();
    }

    vector<3> v;
    vector<3> bg;
    vector<3> ba;
};

// struct Patch {
//     vector<3> p;
//     vector<3> n;
//     std::array<std::array<double, 5>, 5> v;
// };

struct LandmarkState {
    LandmarkState() {
        inv_depth = 0;
        quality = 0;
        plane_id = nil();
    }

    double inv_depth;
    double quality;
    size_t plane_id; // best plane id
    // std::unique_ptr<Patch> patch;
};

struct PlaneState {
    vector<3> normal;
    double distance;
    vector<3> reference_point;
};

} // namespace pvio

#endif // PVIO_STATE_H
