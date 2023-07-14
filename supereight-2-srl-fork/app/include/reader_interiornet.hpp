/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_READER_INTERIORNET_HPP
#define SE_READER_INTERIORNET_HPP



#include <Eigen/Dense>
#include <cstdint>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#include "reader_base.hpp"
#include "se/image/image.hpp"



namespace se {



/** Reader for the InteriorNet dataset.
 * https://interiornet.org/
 */
class InteriorNetReader : public Reader {
    public:
    /** Construct a InteriorNetReader from a ReaderConfig.
     *
     * \param[in] c The configuration struct to use.
     */
    InteriorNetReader(const ReaderConfig& c);

    /** Restart reading from the beginning. */
    void restart();


    /** The name of the reader.
     *
     * \return The string `"InteriorNetReader"`.
     */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

    ReaderStatus nextRay(Eigen::Vector3f& ray_measurement){ return se::ReaderStatus::error; }
    ReaderStatus nextRayPose(Eigen::Matrix4f& T_WB){ return se::ReaderStatus::error; }
    ReaderStatus nextRayBatch(const float batch_interval,
                              std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch){
        return se::ReaderStatus::error;
    }

    static constexpr float interiornet_inverse_scale_ = 1.0f / 1000.0f;
    float inverse_scale_;

    // TODO Allow setting the max_match_timestamp_dist_ and
    // max_interp_timestamp_dist_ at runtime from the YAML file. Not sure how
    // to handle this yet since they only apply to the TUM dataset reader.
    static constexpr double max_match_timestamp_dist_ = 0.02;

    static constexpr double max_interp_timestamp_dist_ = 10.0 * max_match_timestamp_dist_;

    cv::Mat projection_inv_;

    std::vector<std::string> depth_filenames_;

    std::vector<std::string> rgb_filenames_;

    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextRGBA(Image<uint32_t>& rgba_image);
};



} // namespace se



#endif //SE_READER_INTERIORNET_HPP
