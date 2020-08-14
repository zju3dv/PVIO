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
#ifndef PVIO_EXTRA_OPENCV_IMAGE_H
#define PVIO_EXTRA_OPENCV_IMAGE_H

#include <ceres/cubic_interpolation.h>
#include <opencv2/opencv.hpp>
#include <pvio/pvio.h>

namespace pvio::extra {

class OpenCvImage : public Image {
  public:
    OpenCvImage();

    size_t width() const override {
        return image.cols;
    }

    size_t height() const override {
        return image.rows;
    }

    size_t level_num() const override {
        return 3;
    }

    double evaluate(const vector<2> &u, int level = 0) const override;
    double evaluate(const vector<2> &u, vector<2> &ddu, int level = 0) const override;

    void detect_keypoints(std::vector<vector<2>> &keypoints, size_t max_points = 0, double keypoint_distance = 0.5) const override;
    void track_keypoints(const Image *next_image, const std::vector<vector<2>> &curr_keypoints, std::vector<vector<2>> &next_keypoints, std::vector<char> &result_status) const override;

    void preprocess() override;
    void correct_distortion(const matrix<3> &intrinsics, const vector<4> &coeffs);

    cv::Mat image;

  private:
    std::vector<cv::Mat> image_pyramid;
    std::vector<cv::Mat> image_levels;
    std::vector<vector<2>> scale_levels;

    typedef ceres::Grid2D<unsigned char, 1> Grid;
    std::vector<Grid> grid_levels;

    typedef ceres::BiCubicInterpolator<Grid> Interpolator;
    std::vector<Interpolator> interpolator_levels;

    static cv::CLAHE *clahe();
    static cv::GFTTDetector *gftt();
    static cv::FastFeatureDetector *fast();
    static cv::ORB *orb();
};

} // namespace pvio::extra

#endif /* PVIO_EXTRA_OPENCV_IMAGE_H */
