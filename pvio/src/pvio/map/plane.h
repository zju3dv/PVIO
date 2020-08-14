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
#ifndef PVIO_PLANE_H
#define PVIO_PLANE_H

#include <pvio/common.h>
#include <pvio/estimation/state.h>
#include <pvio/map/track.h>
#include <pvio/utility/identifiable.h>
#include <pvio/utility/sector_area.h>

namespace pvio {

class Track;

class Plane : public Identifiable<Plane> {
  public:
    void merge(Plane *plane);
    double overlap_ratio(Plane *plane) const;
    void update_sector_area();
    void update_parameter(bool use_ransac);
    double point_to_plane_abs_distance(const vector<3> &point) const;
    double cast_to_depth(const vector<3> &origin, const vector<3> &direction) const;
    vector<3> cast_to_point(const vector<3> &origin, const vector<3> &direction) const;
    vector<3> point_project_to_plane(const vector<3> &point) const;
    bool is_parallel(const vector<3> &direction, double angle = 10) const;

    PlaneState parameter;
    SectorArea<12> sector_area;
    std::set<Track *, compare<Track *>> tracks;
};

} // namespace pvio

#endif // PVIO_PLANE_H
