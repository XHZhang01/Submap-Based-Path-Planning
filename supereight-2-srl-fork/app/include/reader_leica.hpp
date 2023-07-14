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

#ifndef __READER_LEICA_HPP
#define __READER_LEICA_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cstdint>
#include <fstream>
#include <string>

#include "reader_base.hpp"
#include "se/common/projection.hpp"
#include "se/image/image.hpp"



namespace se {

/** Reader for Leica style datasets.
 * Lidar csv ts | x | y | z | intensity
 */
class LeicaReader : public Reader {
    public:
    /** Construct an NewerCollegeReader from a ReaderConfig.
     *
     * \param[in] c The configuration struct to use.
     */
    LeicaReader(const ReaderConfig& c);

    /** Restart reading from the beginning. */
    void restart();

    /** The name of the reader.
     *
     * \return The string `"LeicaReader"`.
     */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    /// LidarStream
    std::ifstream lidarStream_;
    uint64_t rayTimestamp_ = 0;
    /// TrajectoryStream
    std::ifstream trajectoryStream_;
    uint64_t tsPrev = 0;
    uint64_t tsCurr = 0;
    Eigen::Vector3f posPrev, posCurr;
    Eigen::Quaternionf oriPrev, oriCurr;

    float azimuth_angular_resolution;
    float elevation_angular_resolution;
    std::vector< std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>> clouds_;


    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextRay(Eigen::Vector3f& ray_measurement);

    ReaderStatus nextRayPose(Eigen::Matrix4f& T_WB);

    ReaderStatus nextRayBatch(const float batch_interval,
                              std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch
                              /*std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& rayBatch,
                              std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poseBatch*/);

    ReaderStatus nextRGBA(Image<uint32_t>& rgba_image);

//    static constexpr int8_t pixel_offset[64] = {
//        0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,
//        12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18,
//        0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18};

    /** Return the filenames of LIDAR scans in PCD formatin the supplied directory.
     * LIDAR scans are considered those whose name conforms to the pattern
     * cloud_XXXXXXXXXX_XXXXXXXXX.pcd where X is a digit 0-9.
     *
     * \param[in] dir The directory inside which to look for PCD files.
     * \return The filenames of the PCD files found in lexicographical order.
     */
    //static std::vector<std::string> getScanFilenames(const std::string& dir);
    static std::string getScanFilename(const std::string& dir);
};

} // namespace se

#endif //__READER_LEICA_HPP


