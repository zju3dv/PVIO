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
#include <dataset_reader.h>
#include <optional>
#include <euroc_dataset_reader.h>
#include <tum_dataset_reader.h>
#include <legacy_sensors_dataset_reader.h>
#include <sensors_dataset_reader.h>

std::optional<std::string> path_from_scheme(const std::string &string, const std::string &pattern) {
    if (string.length() >= pattern.length()) {
        if (string.substr(0, pattern.length()) == pattern) {
            return string.substr(pattern.length());
        }
    }
    return {};
}

std::unique_ptr<DatasetReader> DatasetReader::create_reader(const std::string &filename) {
    if (auto path = path_from_scheme(filename, "sensors://")) {
        return std::make_unique<SensorsDatasetReader>(path.value());
    } else if (auto path = path_from_scheme(filename, "legacy-sensors://")) {
        return std::make_unique<LegacySensorsDatasetReader>(path.value());
    } else if (auto path = path_from_scheme(filename, "euroc://")) {
        return std::make_unique<EurocDatasetReader>(path.value());
    } else if (auto path = path_from_scheme(filename, "tum://")) {
        return std::make_unique<TUMDatasetReader>(path.value()); 
    } else {
        return nullptr;
    }
}
