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
#ifndef PVIO_PC_DATASET_READER_H
#define PVIO_PC_DATASET_READER_H

#include <pvio/pvio.h>

class DatasetReader {
  public:
    enum NextDataType {
        AGAIN,
        CAMERA,
        GYROSCOPE,
        ACCELEROMETER,
        END
    };
    virtual ~DatasetReader() = default;
    virtual NextDataType next() = 0;
    virtual std::shared_ptr<pvio::Image> read_image() = 0;
    virtual std::pair<double, pvio::vector<3>> read_gyroscope() = 0;
    virtual std::pair<double, pvio::vector<3>> read_accelerometer() = 0;

    static std::unique_ptr<DatasetReader> create_reader(const std::string &filename);
};

#endif // PVIO_PC_DATASET_READER_H
