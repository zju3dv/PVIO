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
#ifndef PVIO_FRAME_H
#define PVIO_FRAME_H

#include <pvio/common.h>
#include <pvio/estimation/state.h>
#include <pvio/estimation/preintegrator.h>
#include <pvio/utility/identifiable.h>

namespace pvio {

class Config;
class Track;
class Map;
class Factor;

struct create_if_empty_t {};

extern create_if_empty_t create_if_empty;

enum class FrameFlag {
    FF_KEYFRAME = 0,
    FF_FIX_POSE,
    FLAG_NUM
};

class Frame : public Flagged<FrameFlag>, public Identifiable<Frame> {
    friend class Track;
    friend class Map;
    struct construct_by_frame_t;
    Map *map;

  public:
    Frame();
    Frame(size_t id, const construct_by_frame_t &construct);
    virtual ~Frame();

    std::unique_ptr<Frame> clone() const;

    size_t keypoint_num() const {
        return keypoints.size();
    }

    void append_keypoint(const vector<2> &keypoint);

    const vector<2> &get_keypoint(size_t keypoint_index) const {
        return keypoints[keypoint_index];
    }

    Track *get_track(size_t keypoint_index) const {
        return tracks[keypoint_index];
    }

    Track *get_track(size_t keypoint_index, const create_if_empty_t &);

    Factor *get_reprojection_factor(size_t keypoint_index) {
        return reprojection_factors[keypoint_index].get();
    }

    Factor *get_preintegration_factor() {
        return preintegration_factor.get();
    }

    void detect_keypoints(Config *config);
    void track_keypoints(Frame *next_frame, Config *config);
    //void track_keypoints(Frame *next_frame, Config *config, std::function<bool(const Track *)> condition);

    PoseState get_pose(const ExtrinsicParams &sensor) const;
    void set_pose(const ExtrinsicParams &sensor, const PoseState &pose);

    bool has_map() const {
        return map != nullptr;
    }

    std::unique_lock<std::mutex> lock() const;

    matrix<3> K;
    matrix<2> sqrt_inv_cov;
    std::shared_ptr<Image> image;

    PoseState pose;
    MotionState motion;
    ExtrinsicParams camera;
    ExtrinsicParams imu;

    PreIntegrator preintegration;

  private:
    std::vector<vector<2>> keypoints;
    std::vector<Track *> tracks;
    std::vector<std::unique_ptr<Factor>> reprojection_factors;
    std::unique_ptr<Factor> preintegration_factor;
};

} // namespace pvio

#endif // PVIO_FRAME_H
