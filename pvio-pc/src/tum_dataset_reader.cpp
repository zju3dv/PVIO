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
#include <tum_dataset_reader.h>
#include <pvio/extra/opencv_image.h>

TUMDatasetReader::TUMDatasetReader(const std::string &tum_path) {
    TUMCameraCsv camera_csv;
    TUMImuCsv imu_csv;

    camera_csv.load(tum_path + "/cam0/data.csv");
    imu_csv.load(tum_path + "/imu0/data.csv");

    for (auto &item : camera_csv.items) {
        printf("read image %lf\n", item.t);
        image_data.emplace_back(item.t, tum_path + "/cam0/data/" + item.filename);
        all_data.emplace_back(item.t, NextDataType::CAMERA);
    }

    for (auto &item : imu_csv.items) {
        pvio::vector<3> gyr = {item.w.x, item.w.y, item.w.z};
        gyroscope_data.emplace_back(item.t, gyr);
        all_data.emplace_back(item.t, NextDataType::GYROSCOPE);

        pvio::vector<3> acc = {item.a.x, item.a.y, item.a.z};
        accelerometer_data.emplace_back(item.t, acc);
        all_data.emplace_back(item.t, NextDataType::ACCELEROMETER);
    }

    std::sort(all_data.begin(), all_data.end(), [](auto &a, auto &b) {
        return a.first < b.first;
    });
    std::sort(image_data.begin(), image_data.end(), [](auto &a, auto &b) {
        return a.first < b.first;
    });
    std::sort(gyroscope_data.begin(), gyroscope_data.end(), [](auto &a, auto &b) {
        return a.first < b.first;
    });
    std::sort(accelerometer_data.begin(), accelerometer_data.end(), [](auto &a, auto &b) {
        return a.first < b.first;
    });
}

DatasetReader::NextDataType TUMDatasetReader::next() {
    if (all_data.empty()) {
        return NextDataType::END;
    }
    auto [t, type] = all_data.front();
    return type;
}

std::shared_ptr<pvio::Image> TUMDatasetReader::read_image() {
    if (image_data.empty()) {
        return nullptr;
    }
    auto [t, filename] = image_data.front();
    cv::Mat img_distorted = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    cv::Mat img;
    if (!image_undistorter) {
        pvio::matrix<3> K;
        K << 190.97847715128717, 0, 254.93170605935475,
            0, 190.9733070521226, 256.8974428996504,
            0, 0, 1;
        std::vector<double> dist_coeffs = {0.0034003170790442797, 0.001766278153469831, -0.00266312569781606, 0.0003299517423931039};
        image_undistorter = std::make_unique<pvio::extra::ImageUndistorter>(img_distorted.cols, img_distorted.rows, K, dist_coeffs, "equidistant");
    }
    image_undistorter->undistort_image(img_distorted, img);
    std::shared_ptr<pvio::extra::OpenCvImage> opencv_image = std::make_shared<pvio::extra::OpenCvImage>();
    opencv_image->t = t;
    opencv_image->image = img;

    all_data.pop_front();
    image_data.pop_front();
    return opencv_image;
}

std::pair<double, pvio::vector<3>> TUMDatasetReader::read_gyroscope() {
    if (gyroscope_data.empty()) {
        return {};
    }
    auto item = gyroscope_data.front();

    all_data.pop_front();
    gyroscope_data.pop_front();
    return item;
}

std::pair<double, pvio::vector<3>> TUMDatasetReader::read_accelerometer() {
    if (accelerometer_data.empty()) {
        return {};
    }
    auto item = accelerometer_data.front();

    all_data.pop_front();
    accelerometer_data.pop_front();
    return item;
}
