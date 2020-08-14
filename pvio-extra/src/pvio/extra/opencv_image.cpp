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
#include <pvio/extra/opencv_image.h>
#include <pvio/extra/poisson_disk_filter.h>

using namespace cv;

namespace pvio::extra {

static std::vector<Point2f> to_opencv(const std::vector<vector<2>> &v) {
    std::vector<Point2f> r(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        r[i].x = (float)v[i].x();
        r[i].y = (float)v[i].y();
    }
    return r;
}

OpenCvImage::OpenCvImage() = default;

double OpenCvImage::evaluate(const vector<2> &u, int level) const {
    double f;
    const vector<2> &s = scale_levels[level];
    vector<2> su{u.x() * s.x(), u.y() * s.y()};
    interpolator_levels[level].Evaluate(su.y(), su.x(), &f);
    return f;
}

double OpenCvImage::evaluate(const vector<2> &u, vector<2> &ddu, int level) const {
    double f;
    const vector<2> &s = scale_levels[level];
    vector<2> su{u.x() * s.x(), u.y() * s.y()};
    interpolator_levels[level].Evaluate(su.y(), su.x(), &f, &ddu.y(), &ddu.x());
    ddu.x() *= s.x();
    ddu.y() *= s.y();
    return f;
}

void OpenCvImage::detect_keypoints(std::vector<vector<2>> &keypoints, size_t max_points, double keypoint_distance) const {
    if (max_points == 0) {
        max_points = 100;
    }

    std::vector<KeyPoint> cvkeypoints;

    gftt()->detect(image, cvkeypoints);

    if (cvkeypoints.size() > 0) {
        std::sort(cvkeypoints.begin(), cvkeypoints.end(), [](const auto &a, const auto &b) {
            return a.response > b.response;
        });
        std::vector<vector<2>> new_keypoints;
        for (size_t i = 0; i < cvkeypoints.size(); ++i) {
            new_keypoints.emplace_back(cvkeypoints[i].pt.x, cvkeypoints[i].pt.y);
        }

        PoissonDiskFilter<2> filter(keypoint_distance);
        filter.preset_points(keypoints);
        filter.insert_points(new_keypoints);

        new_keypoints.erase(
            std::remove_if(
                new_keypoints.begin(), new_keypoints.end(),
                [this](const auto &keypoint) {
                    return keypoint.x() < 20 || keypoint.y() < 20 || keypoint.x() >= image.cols - 20 || keypoint.y() >= image.rows - 20;
                }),
            new_keypoints.end());

        keypoints.insert(keypoints.end(), new_keypoints.begin(), new_keypoints.end());
    }
}

void OpenCvImage::track_keypoints(const Image *next_image, const std::vector<vector<2>> &curr_keypoints, std::vector<vector<2>> &next_keypoints, std::vector<char> &result_status) const {
    std::vector<Point2f> curr_cvpoints = to_opencv(curr_keypoints);
    std::vector<Point2f> next_cvpoints;
    if (next_keypoints.size() > 0) {
        next_cvpoints = to_opencv(next_keypoints);
    } else {
        next_keypoints.resize(curr_keypoints.size());
        next_cvpoints = curr_cvpoints;
    }

    const OpenCvImage *next_cvimage = dynamic_cast<const OpenCvImage *>(next_image);

    result_status.resize(curr_keypoints.size(), 0);
    if (next_cvimage && curr_cvpoints.size() > 0) {
        Mat cvstatus, cverr;
        calcOpticalFlowPyrLK(image_pyramid, next_cvimage->image_pyramid, curr_cvpoints, next_cvpoints, cvstatus, cverr, Size(21, 21), (int)level_num(), TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01), OPTFLOW_USE_INITIAL_FLOW);
        for (size_t i = 0; i < next_cvpoints.size(); ++i) {
            result_status[i] = cvstatus.at<unsigned char>((int)i);
            if (next_cvpoints[i].x < 20 || next_cvpoints[i].x >= image.cols - 20 || next_cvpoints[i].y < 20 || next_cvpoints[i].y >= image.rows - 20) {
                result_status[i] = 0;
            }
        }
    }

    std::vector<size_t> l;
    std::vector<Point2f> p, q;
    for (size_t i = 0; i < result_status.size(); ++i) {
        if (result_status[i] != 0) {
            l.push_back(i);
            p.push_back(curr_cvpoints[i]);
            q.push_back(next_cvpoints[i]);
        }
    }
    if (l.size() >= 8) {
        Mat mask;
        findFundamentalMat(p, q, cv::FM_RANSAC, 1.0, 0.99, mask);
        for (size_t i = 0; i < l.size(); ++i) {
            if (mask.at<unsigned char>((int)i) == 0) {
                result_status[l[i]] = 0;
            }
        }
    }
    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        if (result_status[i]) {
            next_keypoints[i].x() = next_cvpoints[i].x;
            next_keypoints[i].y() = next_cvpoints[i].y;
        }
    }
}

void OpenCvImage::preprocess() {
    clahe()->apply(image, image);
    image_pyramid.clear();
    image_levels.clear();
    grid_levels.clear();
    interpolator_levels.clear();

    buildOpticalFlowPyramid(image, image_pyramid, Size(21, 21), (int)level_num(), true);
    for (size_t l = 0; l <= level_num(); ++l) {
        image_levels.emplace_back(image_pyramid[l * 2].clone());
    }
    for (size_t l = 0; l <= level_num(); ++l) {
        double sx = (image.cols - 1) / (image_levels[l].cols - 1);
        double sy = (image.rows - 1) / (image_levels[l].rows - 1);
        scale_levels.emplace_back(1.0 / sx, 1.0 / sy);
    }
    for (size_t l = 0; l <= level_num(); ++l) {
        grid_levels.emplace_back(image_levels[l].data, 0, image_levels[l].rows, 0, image_levels[l].cols);
    }
    for (size_t l = 0; l <= level_num(); ++l) {
        interpolator_levels.emplace_back(grid_levels[l]);
    }
}

void OpenCvImage::correct_distortion(const matrix<3> &intrinsics, const vector<4> &coeffs) {
    Mat new_image;
    Mat K(3, 3, CV_32FC1), cvcoeffs(1, 4, CV_32FC1);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K.at<float>(i, j) = (float)intrinsics(i, j);
        }
    }
    for (int i = 0; i < 4; ++i) {
        cvcoeffs.at<float>(i) = (float)coeffs(i);
    }
    undistort(image, new_image, K, cvcoeffs);
    image = new_image;
}

CLAHE *OpenCvImage::clahe() {
    static Ptr<CLAHE> s_clahe = createCLAHE(6.0, cv::Size(8, 8));
    return s_clahe.get();
}

GFTTDetector *OpenCvImage::gftt() {
    static Ptr<GFTTDetector> s_gftt = GFTTDetector::create(1000, 1.0e-3, 20, 3, true);
    return s_gftt.get();
}

FastFeatureDetector *OpenCvImage::fast() {
    static Ptr<FastFeatureDetector> s_fast = FastFeatureDetector::create();
    return s_fast.get();
}

ORB *OpenCvImage::orb() {
    static Ptr<ORB> s_orb = ORB::create();
    return s_orb.get();
}

} // namespace pvio::extra
