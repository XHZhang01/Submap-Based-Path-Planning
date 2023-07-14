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
 *  Created on: Aug 22, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file Measurements.hpp
 * @brief This file contains the templated measurement structs, structs encapsulating
 *        Sensor data and related typedefs.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_MEASUREMENTS_HPP_
#define INCLUDE_OKVIS_MEASUREMENTS_HPP_

#include <deque>
#include <vector>
#include <memory>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/core.hpp>
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#include <Eigen/Dense>
#include <okvis/Time.hpp>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/**
 * \brief Generic measurements
 *
 * They always come with a timestamp such that we can perform
 * any kind of asynchronous operation.
 * \tparam MEASUREMENT_T Measurement data type.
 */
template<class MEASUREMENT_T>
struct Measurement {
  okvis::Time timeStamp;      ///< Measurement timestamp
  MEASUREMENT_T measurement;  ///< Actual measurement.
  int sensorId = -1;          ///< Sensor ID. E.g. camera index in a multicamera setup

  /// \brief Copy constructor.
  /// \param other The other measurement.
  Measurement(const Measurement<MEASUREMENT_T>& other) noexcept :
    timeStamp(other.timeStamp), measurement(other.measurement), sensorId(other.sensorId)
  {

  }

  /// \brief Assignment operator.
  /// \param other The other measurement.
  /// \return This.
  Measurement<MEASUREMENT_T>& operator=(const Measurement<MEASUREMENT_T>& other) noexcept {
    timeStamp = other.timeStamp;
    measurement = other.measurement;
    sensorId = other.sensorId;
    return *this;
  }

  /// \brief Default constructor.
  Measurement()
      : timeStamp(0.0) {
  }
  /**
   * @brief Constructor
   * @param timeStamp_ Measurement timestamp.
   * @param measurement_ Actual measurement.
   * @param sensorId Sensor ID (optional).
   */
  Measurement(const okvis::Time& timeStamp_, const MEASUREMENT_T& measurement_,
              int sensorId = -1)
      : timeStamp(timeStamp_),
        measurement(measurement_),
        sensorId(sensorId) {
  }
};

/// \brief IMU measurements. For now assume they are synchronized:
struct ImuSensorReadings {
  /// \brief Default constructor.
  ImuSensorReadings()
      : gyroscopes(),
        accelerometers() {
  }
  /**
   * @brief Constructor.
   * @param gyroscopes_ Gyroscope measurement.
   * @param accelerometers_ Accelerometer measurement.
   */
  ImuSensorReadings(const Eigen::Matrix<double,3,1,Eigen::Unaligned>& gyroscopes_,
                    const Eigen::Matrix<double,3,1,Eigen::Unaligned>& accelerometers_)
      : gyroscopes(gyroscopes_),
        accelerometers(accelerometers_) {
  }
  Eigen::Matrix<double,3,1,Eigen::Unaligned> gyroscopes;     ///< Gyroscope measurement.
  Eigen::Matrix<double,3,1,Eigen::Unaligned> accelerometers; ///< Accelerometer measurement.
};

/// \brief LiDAR measurements. For now assume they are synchronized:
struct LidarSensorReadings {
    /// \brief Default constructor.
    LidarSensorReadings()
            : rayMeasurement(),
              intensity() {
    }
    /**
     * @brief Constructor.
     * @param rayMeasurement_ lidar measurement.
     * @param intensity_ intensity measurement.
     */
    LidarSensorReadings(const Eigen::Matrix<double,3,1,Eigen::Unaligned>& rayMeasurement_,
                        unsigned int& intensity_)
            : rayMeasurement(rayMeasurement_),
              intensity(intensity_) {
    }
    LidarSensorReadings(const Eigen::Matrix<double,3,1,Eigen::Unaligned>& rayMeasurement_)
            : rayMeasurement(rayMeasurement_),
              intensity(0) {
    }

    Eigen::Matrix<double,3,1,Eigen::Unaligned> rayMeasurement;  ///< Ray in cartesian coordinates.
    unsigned int intensity; ///< Intensity measurement.
};

/// \brief Depth camera measurements. For now assume they are synchronized:
struct DepthCameraData {
  cv::Mat image;  ///< Grayscale/RGB image.
  cv::Mat depthImage; ///< Depth image.
  std::vector<cv::KeyPoint> keypoints;  ///< Keypoints if available.
  bool deliversKeypoints; ///< Are keypoints already delievered in measurement?
};


// this is how we store raw measurements before more advanced filling into data structures happens:
/// \brief IMU measurement.
typedef Measurement<ImuSensorReadings> ImuMeasurement;

/// \brief IMU measurement queue.
typedef std::deque<ImuMeasurement, Eigen::aligned_allocator<ImuMeasurement> > ImuMeasurementDeque;

/// \brief Lidar measurement.
typedef Measurement<LidarSensorReadings> LidarMeasurement;

/// \brief IMU measurement queue.
typedef std::deque<LidarMeasurement, Eigen::aligned_allocator<LidarMeasurement> > LidarMeasurementDeque;

/// \brief Camera measurement.
struct CameraData {
  cv::Mat image;  ///< Image.
  cv::Mat depthImage;  ///< Image.
  std::vector<cv::KeyPoint> keypoints; ///< Keypoints if available.
  bool deliversKeypoints; ///< Are the keypoints delivered too?
};
/// \brief Keypoint measurement.
struct KeypointData {
  std::vector<cv::KeyPoint> keypoints;  ///< Keypoints.
  std::vector<long unsigned int> landmarkIds; ///< Associated landmark IDs.
  cv::Mat descriptors;  ///< Keypoint descriptors.
};
/// \brief Frame measurement.
struct FrameData {
  /// \brief Shared pointer to FrameData.
  typedef std::shared_ptr<okvis::FrameData> Ptr;
  CameraData image; ///< Camera measurement, i.e., image.
  KeypointData keypoints; ///< Keypoints.
};

/// \brief Camera measurement struct.
typedef Measurement<CameraData> CameraMeasurement;

/// \brief GPS measurements.
struct GpsSensorReadings {
  /// \brief Default constructor.
  GpsSensorReadings()
      : position(),
        covariances() {
  }
  /**
   * @brief Constructor from Cartesian coordinates.
   * @param position GPS position measurement.
   * @param covariances GPS measurement covariances.
   */
  GpsSensorReadings(const Eigen::Matrix<double,3,1,Eigen::Unaligned>& position_,
                    const Eigen::Matrix<double,3,3,Eigen::Unaligned>& covariances_)
      : position(position_),
        covariances(covariances_){
    // initialize geodetic coordinates to invalid values
    latitude = -1.;
    longitdue = -1.;
    height = -1.;
    horizontalAccuracy = -1.;
    verticalAccuracy = -1.;
  }
  /**
   * @brief Alternative Constructor from Cartesian coordinates.
   * @param position_ GPS position measurement.
   * @param sigma_x / sigma_y / sigma_z GPS measurement covariances per axis.
  */
  GpsSensorReadings(const Eigen::Matrix<double,3,1,Eigen::Unaligned>& position_,
                    const double sigma_x,
                    const double sigma_y,
                    const double sigma_z){
    this->position = position_;
    Eigen::Matrix<double,3,3,Eigen::Unaligned> covMat;
    covMat.setIdentity();
    covMat(0,0) = sigma_x * sigma_x;
    covMat(1,1) = sigma_y * sigma_y;
    covMat(2,2) = sigma_z * sigma_z;
    this->covariances = covMat;
  }
  /**
   * @brief Constructor from geodetic coordinates.
   * @param lat Latitude Measurement.
   * @param lon Longitdute Measurement.
   * @param height Height Measurement.
   * @param hAcc Horizontal Measurement Error.
   * @param vAcc Vertical Measurement Error.
   */
  GpsSensorReadings(double lat, double lon, double height,
                    double hAcc, double vAcc)
          : latitude(lat), longitdue(lon), height(height),
            horizontalAccuracy(hAcc), verticalAccuracy(vAcc){
    // initialize cartesian coordinates to default invalid values
    position = Eigen::Matrix<double,3,1,Eigen::Unaligned>(0., 0., 0.);
    Eigen::Matrix<double,3,3,Eigen::Unaligned> covMat;
    covMat.setIdentity();
    covMat(0,0) = horizontalAccuracy * horizontalAccuracy;
    covMat(1,1) = horizontalAccuracy * horizontalAccuracy;
    covMat(2,2) = verticalAccuracy * verticalAccuracy;
    this->covariances = covMat;
  }

  void setPosition(double x, double y, double z){
    position.x() = x;
    position.y() = y;
    position.z() = z;
  }

  Eigen::Matrix<double,3,1,Eigen::Unaligned> position;     ///< position measurement.
  Eigen::Matrix<double,3,3,Eigen::Unaligned> covariances;     ///< covariances measurement.
  //Eigen::Vector3d position;     ///< position measurement.
  //Eigen::Matrix3d covariances; ///< covariances measurement.
  double latitude; ///< Geodetic GPS measurements.
  double longitdue; ///< Geodetic GPS measurements.
  double height; ///< Geodetic GPS measurements.
  double horizontalAccuracy; ///< Geodetic GPS measurements.
  double verticalAccuracy; ///< Geodetic GPS measurements.

};
typedef Measurement<GpsSensorReadings> GpsMeasurement;
typedef std::deque<GpsMeasurement, Eigen::aligned_allocator<GpsMeasurement> > GpsMeasurementDeque;

}  // namespace okvis

#endif // INCLUDE_OKVIS_MEASUREMENTS_HPP_
