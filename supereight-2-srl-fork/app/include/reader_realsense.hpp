//
// Created by boche on 5/4/22.
//
/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020 Marija Popovic
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __READER_REALSENSE_HPP
#define __READER_REALSENSE_HPP

#include <Eigen/Dense>
#include <cstdint>
#include <fstream>
#include <string>

#include "reader_base.hpp"
#include "se/common/projection.hpp"
#include "se/image/image.hpp"

#include <srl/sensors/Realsense.hpp>
#include <srl/sensors/RealsenseRgbd.hpp>
#include <srl/queue/threadsafe/ThreadsafeQueue.hpp>

namespace se {

/** Reader for Intel Realsense.
 */
class RealsenseReader : public Reader {
    public:
    /** Construct an RealsenseReader from a ReaderConfig.
     *
     * \param[in] c The configuration struct to use.
     */
    RealsenseReader(const ReaderConfig& c);

    /** Restart reading from the beginning. */
    void restart();

    /** The name of the reader.
     *
     * \return The string `"RealsenseReader"`.
     */
    std::string name() const;

    /** Add RGB Images from Realsense to queue*/
    bool addRgbImage(const srl::utils::Time &stamp, const cv::Mat &rgb);

    /** Add Depth image from Realsense to queue*/
    bool addDepthImage(const srl::utils::Time &stamp, const cv::Mat &depth);

    /** Add IMU measurements from Realsense to queue*/
    bool addImuMeasurement(const srl::utils::Time &stamp, const Eigen::Vector3d &accel,
                                   const Eigen::Vector3d &gyro){return true;}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    float inverse_scale_ = 1. / 1000.; // ToDO: Clarify this

    template<typename T, class Allocator = std::allocator<T>>
    using MyQueue = srl::utils::queue::threadsafe::ThreadSafeQueue<T, Allocator>;
    //std::deque<srl::sensors::DepthCameraMeasurement> depthQueue_;
    //std::deque<srl::sensors::CameraMeasurement> rgbQueue_;
    MyQueue<srl::sensors::DepthCameraMeasurement> depthQueue_;
    MyQueue<srl::sensors::CameraMeasurement> rgbQueue_;


    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextRGBA(Image<uint32_t>& rgba_image);

    ReaderStatus nextRay(Eigen::Vector3f& ray_measurement){ return se::ReaderStatus::error; }
    ReaderStatus nextRayPose(Eigen::Matrix4f& T_WB){ return se::ReaderStatus::error; }
    ReaderStatus nextRayBatch(const float batch_interval,
                              std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch){
        return se::ReaderStatus::error;
    }
};

} // namespace se

#endif //__READER_REALSENSE_HPP


