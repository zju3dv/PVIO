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
#ifndef PVIO_PC_LEGACY_SENSORS_DATASET_READER_H
#define PVIO_PC_LEGACY_SENSORS_DATASET_READER_H

#include <deque>
#include <fstream>
#include <dataset_reader.h>

namespace libsensors {
class Sensors;
}

class LegacySensorsDataParser;

class LegacySensorsDatasetReader : public DatasetReader {
  public:
    LegacySensorsDatasetReader(const std::string &filename);
    ~LegacySensorsDatasetReader();
    NextDataType next() override;
    std::shared_ptr<pvio::Image> read_image() override;
    std::pair<double, pvio::vector<3>> read_gyroscope() override;
    std::pair<double, pvio::vector<3>> read_accelerometer() override;

  private:
    friend class LegacySensorsDataParser;
    std::ifstream datafile;
    std::unique_ptr<libsensors::Sensors> sensors;
    std::deque<std::shared_ptr<pvio::Image>> pending_images;
    std::deque<std::pair<double, pvio::vector<3>>> pending_gyroscopes;
    std::deque<std::pair<double, pvio::vector<3>>> pending_accelerometers;
};

#endif // PVIO_PC_LEGACY_SENSORS_DATASET_READER_H
