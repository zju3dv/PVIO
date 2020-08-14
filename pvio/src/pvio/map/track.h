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
#ifndef PVIO_TRACK_H
#define PVIO_TRACK_H

#include <pvio/common.h>
#include <pvio/estimation/state.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/utility/identifiable.h>

namespace pvio {

enum class TrackFlag {
    TF_VALID = 0,
    TF_TRIANGULATED,
    TF_PLANE,
    FLAG_NUM
};

class Track : public Flagged<TrackFlag>, public Identifiable<Track> {
    friend class Map;
    size_t map_index;
    Map *map;
    Track();

  public:
    Track(const Map::construct_by_map_t &) :
        Track() {
    }

    virtual ~Track();

    size_t keypoint_num() const {
        return keypoint_refs.size();
    }

    std::pair<Frame *, size_t> first_keypoint() const {
        return *keypoint_refs.begin();
    }

    std::pair<Frame *, size_t> last_keypoint() const {
        return *keypoint_refs.rbegin();
    }

    Frame *first_frame() const {
        return keypoint_refs.begin()->first;
    }

    Frame *last_frame() const {
        return keypoint_refs.rbegin()->first;
    }

    const std::map<Frame *, size_t, compare<Frame *>> &keypoint_map() const {
        return keypoint_refs;
    }

    bool has_keypoint(Frame *frame) const {
        return keypoint_refs.count(frame) > 0;
    }

    size_t get_keypoint_index(Frame *frame) const {
        if (has_keypoint(frame)) {
            return keypoint_refs.at(frame);
        } else {
            return nil();
        }
    }

    const vector<2> &get_keypoint(Frame *frame) const;
    void add_keypoint(Frame *frame, size_t keypoint_index);
    void remove_keypoint(Frame *frame, bool suicide_if_empty = true);

    bool triangulate();
    bool try_triangulate(vector<3> &p);

    double compute_parallax() const;
    double compute_baseline() const;

    vector<3> get_landmark_point() const;
    void set_landmark_point(const vector<3> &p);

    std::unique_lock<std::mutex> lock() const {
        return map->lock();
    }

    LandmarkState landmark;
    size_t life;

  private:
    std::map<Frame *, size_t, compare<Frame *>> keypoint_refs;
};

} // namespace pvio

#endif // PVIO_TRACK_H
