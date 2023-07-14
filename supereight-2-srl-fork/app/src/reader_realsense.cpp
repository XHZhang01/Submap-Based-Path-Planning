/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020 Marija Popovic
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "reader_realsense.hpp"

#include <cassert>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <regex>
#include <thread>

#include "se/common/filesystem.hpp"
#include "se/common/image_utils.hpp"

#include <opencv2/opencv.hpp>

se::RealsenseReader::RealsenseReader(const ReaderConfig& c) : se::Reader(c)
{
    std::cout << "Initializing Realsense reader ..." << std::endl;

    // ToDo: Resolution
    // ToDo: how to initialize non-default parameters

    // Set the depth and RGBA image resolutions.
    //depth_image_res_ = Eigen::Vector2i(c.width, c.height);
    //rgba_image_res_ = Eigen::Vector2i(c.width, c.height);

}



void se::RealsenseReader::restart()
{
    // ToDo: need to reset Reader Stream here?
    se::Reader::restart();
    if (stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::ok;
    }
    else {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ << " is not a directory\n";
    }
}



std::string se::RealsenseReader::name() const
{
    return std::string("RealsenseReader");
}

bool se::RealsenseReader::addDepthImage(const srl::utils::Time& stamp, const cv::Mat& depth)
{
    //std::cout << "DEPTH IMAGE " << std::endl;
    //cv::imshow("depth image from callback", depth);
    //cv::waitKey();

    srl::sensors::DepthCameraMeasurement depth_measurement;

    depth_measurement.measurement.depthImage = depth;
    depth_measurement.measurement.deliversKeypoints = false;
    depth_measurement.timeStamp = stamp;
    depth_measurement.sensorId = 0;
    //depthQueue_.push_back(depth_measurement);

    const int depthQueueSize = 100;
    if (depthQueue_.PushNonBlockingDroppingIfFull(depth_measurement, size_t(depthQueueSize))) {
        LOG(WARNING) << "Depth measurement drop";
        return false;
    }
    return true;
}

bool se::RealsenseReader::addRgbImage(const srl::utils::Time& stamp, const cv::Mat& rgb)
{
    //std::cout << "RGB IMAGE " << std::endl;

    srl::sensors::CameraMeasurement rgb_measurement;

    rgb_measurement.measurement.image = rgb;
    rgb_measurement.timeStamp = stamp;
    rgb_measurement.sensorId = 0;
    rgb_measurement.measurement.deliversKeypoints = false;
    //rgbQueue_.push_back(rgb_measurement);

    const int rgbQueueSize = 100;
    if (rgbQueue_.PushNonBlockingDroppingIfFull(rgb_measurement, size_t(rgbQueueSize))) {
        LOG(WARNING) << "RGB measurement drop";
        return false;
    }

    return true;
}

se::ReaderStatus se::RealsenseReader::nextDepth(se::Image<float>& depth_image)
{
    // Get depth image from queue.
    srl::sensors::DepthCameraMeasurement depthMeasurement;

    if(/*depthQueue_.size() == 0*/ !depthQueue_.PopNonBlocking(&depthMeasurement))
        return se::ReaderStatus::skip;

    cv::Mat depth_data;
    depthMeasurement.measurement.depthImage.convertTo(depth_data, CV_32FC1, inverse_scale_);

    cv::Mat wrapper_mat(depth_data.rows, depth_data.cols, CV_32FC1, depth_image.data());
    depth_data.copyTo(wrapper_mat);

    //depthMeasurement = depthQueue_.front();
    //depthQueue_.pop_front();

    // Assign measurement to depth_image output
    //cv::Mat wrapper_mat(depthMeasurement.measurement.depthImage.rows, depthMeasurement.measurement.depthImage.cols,
    //                    CV_32FC1, depth_image.data());
    //depthMeasurement.measurement.depthImage.copyTo(wrapper_mat);

    /*cv::imshow("depth image from callback2", depthMeasurement.measurement.depthImage);
    double min, max;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(depthMeasurement.measurement.depthImage, &min, &max, &minLoc, &maxLoc);
    std::cout << min << " , " << max << std::endl;
    std::cout << depthMeasurement.measurement.depthImage.type() << std::endl;
    cv::waitKey();*/


    return se::ReaderStatus::ok;
}
se::ReaderStatus se::RealsenseReader::nextRGBA(se::Image<uint32_t>& rgba_image)
{
    // Get RGB image from queue.
    srl::sensors::CameraMeasurement rgb;
    if(/*rgbQueue_.size() == 0 */ !rgbQueue_.PopNonBlocking(&rgb))
        return se::ReaderStatus::skip;

    //rgb = rgbQueue_.front();
    //rgbQueue_.pop_front();

    cv::Mat rgba_data;
    cv::cvtColor(rgb.measurement.image, rgba_data, cv::COLOR_BGR2RGBA);

    //assert(rgba_image_res_.x() == static_cast<int>(rgba_data.cols));
    //assert(rgba_image_res_.y() == static_cast<int>(rgba_data.rows));

    cv::Mat wrapper_mat(rgba_data.rows, rgba_data.cols, CV_8UC4, rgba_image.data());
    rgba_data.copyTo(wrapper_mat);

    return se::ReaderStatus::ok;

}
