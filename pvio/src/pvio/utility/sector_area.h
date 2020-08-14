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
#ifndef PVIO_SECTOR_AREA_H
#define PVIO_SECTOR_AREA_H

#include <pvio/pvio.h>
#include <pvio/common.h>

namespace pvio {

template <size_t bin_size>
class SectorArea {
  public:
    SectorArea() {
        bin.fill({0, vector<3>::Zero()});
    }
    virtual ~SectorArea() = default;
    const vector<3> &center() const {
        return cog;
    }
    const vector<3> &normal() const {
        return area_normal;
    }
    const std::array<std::pair<double, vector<3>>, bin_size> &sectors() {
        return bin;
    }
    void update_parameter(const vector<3> &center, const vector<3> &normal, bool preserve_area = false) {
        cog = center;
        basis = s2_tangential_basis_barrel(normal);
        area_normal = normal;
        std::array<std::pair<double, vector<3>>, bin_size> old_bin;
        if (preserve_area) {
            old_bin = bin;
            bin.fill({0, vector<3>::Zero()});
            for (const auto &sector : old_bin) {
                if (sector.first > 0) insert_point(sector.second);
            }
        } else {
            bin.fill({0, vector<3>::Zero()});
        }
    }
    void insert_point(const vector<3> &x) {
        vector<2> projection;
        size_t bin_index = compute_bin_index(x, projection);
        double distance = projection.norm();
        vector<3> x_in_area = x - (area_normal.dot(x - cog)) * area_normal;
        if (max_radius(bin_index) < distance) {
            extend_sector(bin_index, distance, x_in_area);
            // extend_sector(bin_index, distance, x);
        }
    }
    bool is_near_boundary(const vector<3> &x, bool insert_if_near = false, const double distance_ratio = 1.2, const double step = 0.1) {
        vector<2> projection;
        size_t bin_index = compute_bin_index(x, projection);
        double distance = projection.norm();
        double curr_radius = max_radius(bin_index);
        if (distance < curr_radius || distance / curr_radius < distance_ratio || (distance - curr_radius) < step) {
            if (insert_if_near) {
                extend_sector(bin_index, distance, x);
            }
            return true;
        } else {
            return false;
        }
    }
    void merge(const SectorArea &slave_area) {
        std::vector<std::pair<double, vector<3>>> sectors;
        sectors.reserve(bin_size * 2);
        sectors.insert(sectors.end(), slave_area.bin.begin(), slave_area.bin.end());
        sectors.insert(sectors.end(), bin.begin(), bin.end());
        // add two cogs
        sectors.emplace_back(0.5, center());
        sectors.emplace_back(0.5, slave_area.center());
        size_t vertex_count = 0;
        vector<3> new_cog = vector<3>::Zero();
        for (const auto &sector : sectors) {
            if (sector.first > 0) {
                new_cog += sector.second;
                vertex_count++;
            }
        }
        new_cog /= double(vertex_count);
        new_cog = new_cog - (area_normal.dot(new_cog - cog)) * area_normal;
        for (const auto &sector : sectors) {
            if (sector.first > 0) {
                insert_point(sector.second);
            }
        }
    }
    void centralize() {
        vector<3> new_cog = vector<3>::Zero();
        size_t count = 0;
        for (const auto &sector : bin) {
            if (sector.first > 0) {
                new_cog += sector.second;
                count++;
            }
        }
        new_cog /= double(count);
        new_cog = new_cog - (area_normal.dot(new_cog - cog)) * area_normal;
        vector<3> new_normal = area_normal;
        update_parameter(new_cog, new_normal, true);
    }

  private:
    double center_distance(vector<3> &x) const {
        return (cog - x).norm();
    }
    size_t compute_bin_index(const vector<3> &x, vector<2> &projection) const {
        projection = basis.transpose() * x;
        double angle = std::atan2(projection.y(), projection.x()) * (180 / M_PI);
        angle += 180;
        return static_cast<size_t>(angle / (360.0 / bin_size));
    }
    double max_radius(size_t bin_index) const {
        return bin[bin_index].first;
    }
    void extend_sector(size_t bin_index, double distance, const vector<3> &x) {
        runtime_assert(bin_index < bin.size() && bin_index >= 0, "extend_sector: invalid bin_index.");
        if (bin[bin_index].first < distance) {
            bin[bin_index] = {distance, x};
        }
    }
    matrix<3, 2> s2_tangential_basis_barrel(const vector<3> &x) const {
        vector<3> b1 = x.cross(abs(x.z()) < 0.866 ? vector<3>::UnitZ() : vector<3>::UnitY()).normalized();
        vector<3> b2 = x.cross(b1).normalized();
        return (matrix<3, 2>() << b1, b2).finished();
    }
    matrix<3, 2> basis;
    vector<3> cog;
    vector<3> area_normal;
    /// <max_distance, vertex with max distance>
    std::array<std::pair<double, vector<3>>, bin_size> bin;
};

}; // namespace pvio

#endif // PVIO_SECTOR_AREA_H
