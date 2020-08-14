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
#include <sensors_dataset_reader.h>
#include <array>
#include <iostream>
#include <libsensors.h>
#include <pvio/extra/opencv_image.h>

class SensorsDataParser : public libsensors::Sensors {
  public:
    SensorsDataParser(SensorsDatasetReader *reader) :
        reader(reader) {
    }

  protected:
    void on_image(double t, int width, int height, const unsigned char *bytes) override {
        std::shared_ptr<pvio::extra::OpenCvImage> image = std::make_shared<pvio::extra::OpenCvImage>();
        image->t = t;
        image->image = cv::Mat(cv::Size(width, height), CV_8UC1, const_cast<unsigned char *>(bytes)).clone();
        reader->pending_images.push_back(image);
    }

    void on_accelerometer(double t, double x, double y, double z) override {
        reader->pending_accelerometers.emplace_back(t, pvio::vector<3>{x, y, z});
    }

    void on_gyroscope(double t, double x, double y, double z) override {
        reader->pending_gyroscopes.emplace_back(t, pvio::vector<3>{x, y, z});
    }

  private:
    SensorsDatasetReader *reader;
};

SensorsDatasetReader::SensorsDatasetReader(const std::string &filename) :
    datafile(filename.c_str(), std::ifstream::in | std::ifstream::binary) {
    if (!datafile) {
        std::cout << "Cannot open file " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
    sensors = std::make_unique<SensorsDataParser>(this);
}

SensorsDatasetReader::~SensorsDatasetReader() = default;

DatasetReader::NextDataType SensorsDatasetReader::next() {
    bool data_available = false;
    double image_time = std::numeric_limits<double>::max();
    double gyroscope_time = std::numeric_limits<double>::max();
    double accelerometer_time = std::numeric_limits<double>::max();
    if (pending_images.size() > 0) {
        image_time = pending_images.front()->t;
        data_available = true;
    }
    if (pending_gyroscopes.size() > 0) {
        gyroscope_time = pending_gyroscopes.front().first;
        data_available = true;
    }
    if (pending_accelerometers.size() > 0) {
        accelerometer_time = pending_accelerometers.front().first;
        data_available = true;
    }
    if (data_available) {
        if (accelerometer_time <= image_time && accelerometer_time <= gyroscope_time) {
            return DatasetReader::ACCELEROMETER;
        } else if (gyroscope_time <= image_time && gyroscope_time < accelerometer_time) {
            return DatasetReader::GYROSCOPE;
        } else {
            return DatasetReader::CAMERA;
        }
    } else {
        std::array<char, 8192> buffer;
        if (!datafile.read(buffer.data(), 8192)) {
            return DatasetReader::END;
        }
        size_t len = datafile.gcount();
        if (len == 0) {
            return DatasetReader::END;
        }
        sensors->parse_data(buffer.data(), len);
        return DatasetReader::AGAIN;
    }
}

std::shared_ptr<pvio::Image> SensorsDatasetReader::read_image() {
    std::shared_ptr<pvio::Image> image = pending_images.front();
    pending_images.pop_front();
    return image;
}

std::pair<double, pvio::vector<3>> SensorsDatasetReader::read_gyroscope() {
    std::pair<double, pvio::vector<3>> gyroscope = pending_gyroscopes.front();
    pending_gyroscopes.pop_front();
    return gyroscope;
}
std::pair<double, pvio::vector<3>> SensorsDatasetReader::read_accelerometer() {
    std::pair<double, pvio::vector<3>> accelerometer = pending_accelerometers.front();
    pending_accelerometers.pop_front();
    return accelerometer;
}
