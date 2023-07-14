/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: April 14, 2020
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file ViSensorBase.hpp
 * @brief Header file for the ViSensorBase class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_VISENSORBASE_HPP_
#define INCLUDE_OKVIS_VISENSORBASE_HPP_

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/Time.hpp>


/// \brief okvis Main namespace of this package.
namespace okvis {

/// @brief Generic sensor interface for different VI sensors to derive from.
class ViSensorBase {
public:
  /// \brief Callback for receiving images.
  typedef std::function<
        bool(const okvis::Time &,
            const std::vector<cv::Mat> &)> ImageCallback;

  /// \brief Callback for receiving IMU measurements.
  typedef std::function<
        bool(const okvis::Time &,
             const Eigen::Vector3d &, const Eigen::Vector3d & )> ImuCallback;
  typedef std::function<
        bool(const okvis::Time &,
             const Eigen::Vector3d &, const Eigen::Vector3d &)> GpsCallback;
  typedef std::function<
          bool(const okvis::Time &,
               const double &, const double &, const double &,
               const double &, const double &)> GeodeticGpsCallback;

  /// \brief Callback for receiving RGB measurements.
  typedef std::function<
          bool(const okvis::Time &,
               const cv::Mat &)> RgbImageCallback;

  /// \brief Callback for receiving depth measurements.
  typedef std::function<
          bool(const okvis::Time &,
                  const cv::Mat &)> DepthImageCallback;

  /// \brief Default destructor.
  virtual ~ViSensorBase() = default;

  /// @brief Set the images callback
  /// @param imagesCallback The images callback to register.
  virtual void setImagesCallback(const ImageCallback& imagesCallback) final {
    imagesCallback_ = imagesCallback;
  }

  /// @brief Set the IMU callback
  /// @param imuCallback The IMU callback to register.
  virtual void setImuCallback(const ImuCallback& imuCallback) final {
    imuCallback_ = imuCallback;
  }

  /// @brief Set the GPS callback
  /// @param gpsCallback The GPS callback to register.
  virtual void setGpsCallback(const GpsCallback& gpsCallback) final {
    gpsCallback_ = gpsCallback;
  }

  /// @brief Set the Geodetic GPS callback
  /// @param geodeticGpsCallback The GPS callback to register.
  virtual void setGeodeticGpsCallback(const GeodeticGpsCallback & geodeticGpsCallback) final {
    geodeticGpsCallback_ = geodeticGpsCallback;
  }
  /// @brief Set the images callback
  /// @param imagesCallback The images callback to register.
  virtual void setRgbImageCallback(const RgbImageCallback & rgbImageCallback) final {
    rgbCallback_ = rgbImageCallback;
  }

  /// @brief Set the images callback
  /// @param imagesCallback The images callback to register.
  virtual void setDepthImageCallback(const DepthImageCallback & depthImageCallback) final {
    depthCallback_ = depthImageCallback;
  }

  /// @brief Starts streaming.
  /// @return True, if successful
  virtual bool startStreaming() = 0;

  /// @brief Stops streaming.
  /// @return True, if successful
  virtual bool stopStreaming() = 0;

  /// @brief Check if currently streaming.
  /// @return True, if streaming.
  virtual bool isStreaming() = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  ImageCallback imagesCallback_; ///< The registered images callback.
  ImuCallback imuCallback_; ///< The registered IMU callback.
  GpsCallback gpsCallback_; ///< The registered GPS callback. // ToDo: rename to cartesian
  GeodeticGpsCallback geodeticGpsCallback_; ///< The registered (geodetic) GPS callback.
  RgbImageCallback rgbCallback_;  ///< The registered callback for the RGB image.
  DepthImageCallback depthCallback_;  ///< The registered callback for the depth image.
};

/// @brief Reader class acting like a VI sensor.
/// @warning Make sure to use this in combination with synchronous
/// processing, as there is no throttling of the reading process.
class DatasetReaderBase : public ViSensorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// @brief (Re-)setting the dataset path.
  /// @param path The absolute or relative path to the dataset.
  /// @return True, if the dateset folder structure could be created.
  virtual bool setDatasetPath(const std::string & path) = 0;

  /// @brief Setting skip duration in the beginning.
  /// deltaT Duration [s] to skip in the beginning.
  virtual bool setStartingDelay(const okvis::Duration & deltaT) = 0;

  /// @brief Get the completion fraction read already.
  /// @return Fraction read already.
  virtual double completion() const = 0;

};

} // okvis

#endif
