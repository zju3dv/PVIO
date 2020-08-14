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
#ifndef PVIO_MAP_H
#define PVIO_MAP_H

#include <pvio/common.h>

namespace pvio {

class Factor;
class Frame;
class Plane;
class Track;

class Map {
    friend class Track;
    struct construct_by_map_t;

  public:
    Map();
    virtual ~Map();

    bool equals(Map *map) const;

    void clear();

    size_t frame_num() const {
        return frames.size();
    }

    Frame *get_frame(size_t index) const {
        return frames[index].get();
    }

    Frame *first_frame() const {
        return frames[0].get();
    }

    Frame *last_frame() const {
        return frames[frames.size() - 1].get();
    }

    void put_frame(std::unique_ptr<Frame> frame, size_t position = nil());

    void erase_frame(size_t index);
    void marginalize_frame(size_t index);

    size_t frame_index_by_id(size_t id) const;

    size_t track_num() const {
        return tracks.size();
    }

    Track *get_track(size_t index) const {
        return tracks[index].get();
    }

    Track *create_track();
    void erase_track(Track *track);

    void prune_tracks(const std::function<bool(const Track *)> &condition);

    Track *get_track_by_id(size_t id) const;

    size_t plane_num() const {
        return planes.size();
    }

    Plane *get_plane(size_t index) const {
        return planes[index].get();
    }

    void put_plane(std::unique_ptr<Plane> plane);

    void erase_plane(size_t index);

    void set_marginalization_factor(std::unique_ptr<Factor> factor);

    Factor *get_marginalization_factor() {
        return marginalization_factor.get();
    }

    std::unique_lock<std::mutex> lock() const {
        return std::unique_lock(map_mutex);
    }

  private:
    void recycle_track(Track *track);

    std::deque<std::unique_ptr<Frame>> frames;
    std::vector<std::unique_ptr<Plane>> planes;
    std::vector<std::unique_ptr<Track>> tracks;
    std::map<size_t, Track *> track_id_map;

    std::unique_ptr<Factor> marginalization_factor;
    mutable std::mutex map_mutex;
};

} // namespace pvio

#endif // PVIO_MAP_H
