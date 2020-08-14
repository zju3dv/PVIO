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
#ifndef PVIO_PLANE_EXTRACTOR_H
#define PVIO_PLANE_EXTRACTOR_H

#include <pvio/estimation/state.h>
#include <pvio/utility/worker.h>

namespace pvio {

class Map;
class Track;

class PlaneExtractor : public Worker {
  public:
    PlaneExtractor(Map *map);
    ~PlaneExtractor();

    bool empty() const override;
    void work(std::unique_lock<std::mutex> &l) override;

    void issue_extraction();
    void update_map();
    void extend_planes_and_cast_plane_points(double extend_rpe_ratio);
    void merge_planes();

    // compute reprojection error with given landmark point
    static double compute_reprojection_error(const Map *map, const Track *track, const vector<3> &point);

    static double enough_baseline(const Track *track);

    struct PlaneSolver {
        PlaneState operator()(const std::array<vector<3>, 3> &points) const {
            PlaneState plane;
            plane.normal = (points[2] - points[0]).cross(points[1] - points[0]);
            plane.normal.stableNormalize();
            plane.distance = points[0].dot(plane.normal);
            return plane;
        }
    };

    struct PlaneEvaluator {
        const PlaneState &plane;
        PlaneEvaluator(const PlaneState &plane) :
            plane(plane) {
        }
        double operator()(const vector<3> &point) const {
            return std::abs(point.dot(plane.normal) - plane.distance);
        }
    };

  private:
    struct PlaneRecord {
        PlaneState parameter;
        std::vector<size_t> track_ids;
    };

    Map *map;
    std::deque<PlaneRecord> plane_records;

    bool extraction_issued = false;
    mutable std::mutex planes_mutex;
};

} // namespace pvio

#endif // PVIO_PLANE_EXTRACTOR_H
