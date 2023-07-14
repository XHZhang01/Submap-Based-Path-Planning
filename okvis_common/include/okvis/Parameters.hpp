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
 *  Created on: Apr 22, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Parameters.hpp
 * @brief This file contains struct definitions that encapsulate parameters and settings.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_PARAMETERS_HPP_
#define INCLUDE_OKVIS_PARAMETERS_HPP_

#include <deque>
#include <vector>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/core.hpp>
#pragma GCC diagnostic pop
#include <Eigen/Dense>
#include <okvis/Time.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// @brief Struct that contains all the camera calibration information.
struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  okvis::kinematics::Transformation T_SC;   ///< Transformation from camera to sensor (IMU) frame.
  Eigen::Vector2i imageDimension;           ///< Image dimension. [pixels]
  Eigen::VectorXd distortionCoefficients;   ///< Distortion Coefficients.
  Eigen::Vector2d focalLength;              ///< Focal length.
  Eigen::Vector2d principalPoint;           ///< Principal point.
  std::string cameraModel;                  ///< camera model. ('pinhole' 'eucm')
  Eigen::Vector2d eucmParameters;           ///< alpha, beta eucm parameters

  /// \brief Distortion type. ('radialtangential' 'radialtangential8' 'equdistant')
  std::string distortionType;
};

/*!
 * \brief Camera parameters.
 *
 * A simple struct to specify properties of a Camera.
 *
 */
struct CameraParameters{
  double timestamp_tolerance; ///<Stereo frame out-of-sync tolerance. [s]

  /// \brief Image timestamp error. [s] timestamp_camera_correct = timestamp_camera - image_delay.
  double image_delay;

  /**
   * @brief Some parameters to set the online calibrator.
   */
  struct OnlineCalibrationParameters {
    bool do_extrinsics; ///< Do we online-calibrate extrinsics?
    double sigma_r; ///< T_SCi position prior stdev [m]
    double sigma_alpha; ///< T_SCi orientation prior stdev [rad]
  };

  OnlineCalibrationParameters online_calibration; ///< Online calibration parameters.
};

/*!
 * \brief IMU parameters.
 *
 * A simple struct to specify properties of an IMU.
 *
 */
struct ImuParameters{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  okvis::kinematics::Transformation T_BS; ///< Transform from Body frame to IMU (sensor frame S).
  double a_max;  ///< Accelerometer saturation. [m/s^2]
  double g_max;  ///< Gyroscope saturation. [rad/s]
  double sigma_g_c;  ///< Gyroscope noise density.
  double sigma_bg;  ///< Initial gyroscope bias.
  double sigma_a_c;  ///< Accelerometer noise density.
  double sigma_ba;  ///< Initial accelerometer bias
  double sigma_gw_c; ///< Gyroscope drift noise density.
  double sigma_aw_c; ///< Accelerometer drift noise density.
  Eigen::Vector3d g0;  ///< Mean of the prior gyro bias.
  Eigen::Vector3d a0;  ///< Mean of the prior accelerometer bias.
  double g;  ///< Earth acceleration.
  Eigen::Vector3d s_a; ///< Scale factor for accelerometer measurements
};

/**
 * @brief Parameters for detection etc.
 */
struct FrontendParameters {
  double detection_threshold; ///< Detection threshold. By default the uniformity radius in pixels.
  double absolute_threshold; ///< Absolute Harris corner threshold (noise floor).
  double matching_threshold; ///< BRISK descriptor matching threshold.
  int octaves; ///< Number of octaves for detection. 0 means single-scale at highest resolution.
  int max_num_keypoints; ///< Restrict to a maximum of this many keypoints per img (strongest ones).
  double keyframe_overlap; ///< Minimum field-of-view overlap.
  bool use_cnn; ///< Use the CNN (if available) to filter out dynamic content / sky.
  bool parallelise_detection; ///< Run parallel detect & describe.
  int num_matching_threads; ///< Parallelise matching with this number of threads.
};

/**
 * @brief Parameters regarding the estimator.
 */
struct EstimatorParameters {
  int num_keyframes; ///< Number of keyframes in optimisation window.
  int num_loop_closure_frames; ///< Number of loop closure frames in optimisation window.
  int num_imu_frames; ///< Number of frames linked by most recent nonlinear IMU error terms.
  bool do_loop_closures; ///< Whether to do VI-SLAM or VIO.
  bool do_final_ba; ///< Whether to run a final full BA.
  bool enforce_realtime; ///< Whether to limit the time budget for optimisation.
  int realtime_min_iterations; ///< Minimum number of iterations always performed.
  int realtime_max_iterations; ///< Never do more than these, even if not converged.
  double realtime_time_limit; ///< Time budget for realtime optimisation. [s]
  int realtime_num_threads; ///< Number of threads for the realtime optimisation.
  int full_graph_iterations; ///< Don't do more than these for the full (background) optimisation.
  int full_graph_num_threads; ///< Number of threads for the full (background) optimisation.
};

/**
 * @brief Some options for how and what to output.
 */
struct OutputParameters {
    bool display_matches; ///< Displays debug video and matches. May be slow.
    bool display_overhead; ///< Debug overhead image. Is slow.

    /// \brief Should the state that is propagated with IMU messages be published or just the
    /// optimised ones?
    bool publish_imu_propagated_state;
    double imu_propagated_state_publishing_rate; ///< Limit propagated state publishing rate
    bool enable_submapping; //< Whether or not is submapping enabled
};
/**
  * @brief Struct to specify parameters of GPS sensor
  */
struct GpsParameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool use_gps; ///< Flag if GPS data is used at all
    std::string type; ///< Format of GPS data: "cartesian" | "geodetic" | "geodetic-leica":
    Eigen::Vector3d r_SA; ///< Translation IMU sensor to GPS antenna; known from calibration
    double gpsObservabilityThreshold; ///< Threshold for heuristic decision whether gps extrinsics are observable
    double gpsMeasVarianceThreshold; /// < Threshold on the chi square error
    double yawErrorThreshold; /// < Threshold on maximum estimated yaw error [degree] for initialization
    int gpsFusionMode; /// < Mode how GPS Fusion is achieved; 1: do svd initialization only once | 2: do SVD initialisation in every step | 3: do position alignment only | 4: do position + full alignment

    /// Default Constructor (no GPS)
    GpsParameters() : use_gps(false), type("none"), r_SA(Eigen::Vector3d(0., 0., 0.)),
                      gpsObservabilityThreshold(0.), gpsMeasVarianceThreshold(0.), yawErrorThreshold(0.),
                      gpsFusionMode(0){}
};



/// @brief Struct to combine all parameters and settings.
struct ViParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  okvis::cameras::NCameraSystem nCameraSystem;  ///< Camera extrinsics and intrinsics.
  CameraParameters camera; ///< Camera parameters.
  ImuParameters imu; ///< Imu parameters.
  GpsParameters gps; ///< Gps parameters.
  FrontendParameters frontend; ///< Frontend parameters.
  EstimatorParameters estimator; ///< Estimator parameters.
  OutputParameters output; ///< Output parameters.
  CameraCalibration rgb;  ///< RGB parameters.
};

} // namespace okvis

#endif // INCLUDE_OKVIS_PARAMETERS_HPP_
