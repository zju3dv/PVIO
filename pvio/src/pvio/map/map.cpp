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
#include <pvio/estimation/bundle_adjustor.h>
#include <pvio/estimation/factor.h>
#include <pvio/map/frame.h>
#include <pvio/map/map.h>
#include <pvio/map/plane.h>
#include <pvio/map/track.h>
#include <pvio/utility/debug.h>

namespace pvio {

struct Map::construct_by_map_t {};

Map::Map() = default;

Map::~Map() = default;

void Map::clear() {
    frames.clear();
    tracks.clear();
}

void Map::put_frame(std::unique_ptr<Frame> frame, size_t position) {
    frame->map = this;
    if (position == nil()) {
        frames.emplace_back(std::move(frame));
        position = frames.size() - 1;
    } else {
        frames.emplace(frames.begin() + position, std::move(frame));
    }
    if (position > 0) {
        Frame *frame_i = frames[position - 1].get();
        Frame *frame_j = frames[position].get();
        runtime_assert(frame_j->id() > frame_i->id(), "Wrong frame order.");
        frame_j->preintegration_factor = Factor::create_preintegration_error(frame_i, frame_j);
    }
    if (position < frames.size() - 1) {
        Frame *frame_i = frames[position].get();
        Frame *frame_j = frames[position + 1].get();
        runtime_assert(frame_j->id() > frame_i->id(), "Wrong frame order.");
        frame_j->preintegration_factor = Factor::create_preintegration_error(frame_i, frame_j);
    }
}

void Map::erase_frame(size_t index) {
    Frame *frame = frames[index].get();
    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        if (Track *track = frame->get_track(i); track != nullptr) {
            track->remove_keypoint(frame);
        }
    }
    frames.erase(frames.begin() + index);
    if (index > 0 && index < frames.size()) {
        Frame *frame_i = frames[index - 1].get();
        Frame *frame_j = frames[index].get();
        frame_j->preintegration_factor = Factor::create_preintegration_error(frame_i, frame_j);
    }
}

void Map::marginalize_frame(size_t index) {
    BundleAdjustor().marginalize_frame(this, index);
    Frame *frame = frames[index].get();
    for (size_t i = 0; i < frame->keypoint_num(); ++i) {
        if (Track *track = frame->get_track(i); track != nullptr) {
            track->remove_keypoint(frame);
        }
    }
    frames.erase(frames.begin() + index);
    if (index > 0 && index < frames.size()) {
        frames[index]->preintegration_factor.reset();
    }
}

size_t Map::frame_index_by_id(size_t id) const {
    struct FrameID {
        FrameID(const std::unique_ptr<Frame> &frame) :
            id(frame->id()) {
        }
        FrameID(size_t id) :
            id(id) {
        }
        bool operator<(const FrameID &fi) const {
            return id < fi.id;
        }
        size_t id;
    };
    auto it = std::lower_bound(frames.begin(), frames.end(), id, std::less<FrameID>());
    if (it == frames.end()) return nil();
    if (id < (*it)->id()) return nil();
    return std::distance(frames.begin(), it);
}

Track *Map::create_track() {
    std::unique_ptr<Track> track = std::make_unique<Track>(construct_by_map_t());
    track->map_index = tracks.size();
    track->map = this;
    track_id_map[track->id()] = track.get();
    tracks.emplace_back(std::move(track));
    return tracks.back().get();
}

void Map::erase_track(Track *track) {
    while (track->keypoint_num() > 0) {
        track->remove_keypoint(track->keypoint_map().begin()->first, false);
    }
    recycle_track(track);
}

void Map::prune_tracks(const std::function<bool(const Track *)> &condition) {
    std::vector<Track *> tracks_to_prune;
    for (size_t i = 0; i < track_num(); ++i) {
        if (Track *track = get_track(i); condition(track)) {
            tracks_to_prune.push_back(track);
        }
    }
    for (const auto &track : tracks_to_prune) {
        erase_track(track);
    }
}

Track *Map::get_track_by_id(size_t id) const {
    if (track_id_map.count(id)) {
        return track_id_map.at(id);
    } else {
        return nullptr;
    }
}

void Map::put_plane(std::unique_ptr<Plane> plane) {
    std::vector<size_t> overlapped;
    for (size_t i = 0; i < planes.size(); ++i) {
        if (abs(plane->parameter.normal.dot(planes[i]->parameter.normal)) < 0.95) continue;
        double overlap_ratio = plane->overlap_ratio(planes[i].get());
        if (overlap_ratio > 0.3) {
            overlapped.push_back(i);
        }
    }
    if (overlapped.empty()) {
        planes.emplace_back(std::move(plane));
    } else {
        for (size_t i = overlapped.size(); i > 1; --i) {
            size_t j = overlapped[i - 1];
            plane->merge(planes[j].get());
            planes.erase(planes.begin() + j);
        }
        planes[overlapped[0]]->merge(plane.get());
    }
}

void Map::erase_plane(size_t index) {
    planes.erase(planes.begin() + index);
}

void Map::set_marginalization_factor(std::unique_ptr<Factor> factor) {
    marginalization_factor = std::move(factor);
}

void Map::recycle_track(Track *track) {
    if (track->map_index != tracks.back()->map_index) {
        tracks[track->map_index].swap(tracks.back());
        tracks[track->map_index]->map_index = track->map_index;
    }
    track_id_map.erase(track->id());
    for (size_t i = 0; i < planes.size(); ++i) {
        planes[i]->tracks.erase(track);
    }
    tracks.pop_back();
}

} // namespace pvio
