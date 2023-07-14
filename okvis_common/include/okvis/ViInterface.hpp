/*****************************************************ucm****************************
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
 *  Created on: Jun 11, 2013
 *      Author: Paul Furgale
 *              Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *              Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file ViInterface.hpp
 * @brief Header file for the ViInterface class.
 * @author Paul Furgale
 * @author Stefan Leutenegger
 * @author Andreas Froster
 */

#ifndef INCLUDE_OKVIS_VIINTERFACE_HPP_
#define INCLUDE_OKVIS_VIINTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <functional>
#include <atomic>
#include <mutex>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#pragma GCC diagnostic pop
#include <okvis/assert_macros.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Measurements.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// @brief Struct to hold all estimated states at a certain time instant.
struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  kinematics::Transformation T_WS; ///< Transformation between World W and Sensor S.
  Eigen::Vector3d v_W; ///< Velocity in frame W [m/s].
  Eigen::Vector3d b_g; ///< Gyro bias [rad/s].
  Eigen::Vector3d b_a; ///< Accelerometer bias [m/s^2].
  Eigen::Vector3d omega_S; ///< Rotational velocity in frame S [rad/s].
  Time timestamp; ///< Timestamp corresponding to this state.
  StateId id; ///< Frame Id.
  ImuMeasurementDeque previousImuMeasurements; ///< IMU measurements up to this state's time.
  bool isKeyframe; ///< Is it a keyframe?
};

/// @brief Simple enum to denote the tracking quality.
enum class TrackingQuality {
  Good, ///< Good quality for at least 30% of the image containing tracks.
  Marginal, ///< Marginal quality below that 30% level.
  Lost ///< Lost, if only one or less keypoints are matched per image.
};

/// @brief A helper struct to hold information about the tracking state.
struct TrackingState {
  StateId id; ///< ID this tracking info refers to.
  bool isKeyframe; ///< Is it a keyframe?
  TrackingQuality trackingQuality; ///< The tracking quality.
  bool recognisedPlace; ///< Has this fram recognised a place / relocalised / loop-closed?
  bool isFullGraphOptimising; ///< Is the background loop closure optimisation currently ongoing?
  StateId currentKeyframeId; ///< The ID of the current keyframe.
};

/// \brief A helper class to store, update and access states
class Trajectory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \name Setters
  /// \{

  /// \brief Update the trajectory.
  /// @param[in] trackingState Tracking state as received from Estimator callback.
  /// @param[in] updatedStates Updated states as received from Estimator callback.
  /// @param[out] affectedStateIds IDs of affected states.
  void update(const TrackingState & trackingState,
              std::shared_ptr<const AlignedMap<StateId, State> > updatedStates,
              std::set<StateId>& affectedStateIds);
  /// \}
  /// \name Getters
  /// \{

  /// \brief Obtain the (extrapolated) state at a certain timestamp. 
  ///        Note that this may access non-keyframe states (relative pose computations)
  ///        and may need to propagate to the precise timestamp. Therefore this is not a
  ///        trivial operation. Returns false, if no states/IMU measurements available for
  ///        the desired timestamp.
  /// @param[in] timestamp The requested timestamp.
  /// @param[out] state The respective state.
  /// \return True on success.
  bool getState(const Time & timestamp, State& state);

  /// \brief Get the state at an actually estimated state instance.
  /// @param[in] stateId The requested state ID.
  /// @param[out] state The respective state.
  /// \return True on success.
  bool getState(const StateId & stateId, State& state) const;

  /// \brief Get all the stateIds (estimated states).
  /// \return All the stateIds (estimated states).
  const std::set<StateId>& stateIds() const {return allStateIds_;}

  /// \brief Get the keyframe states.
  /// \return An std::map from keyframe state IDs to keyframe states.
  const AlignedMap<StateId, State>& getKeyframes() const;

  /// \}
 private:
  AlignedMap<StateId, State> keyframeStates_; ///< Stores the keyframe states.
  std::map<StateId, StateId> keyframeIdByStateId_; ///< The reference keyframe Id by State Id.
  std::set<StateId> allStateIds_; ///< All stateIds.

  /// \brief helper struct for non-keyframe states.
  struct NonKeyframeState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    kinematics::Transformation T_Sk_S; ///< Transformation between keyframe Sk and Sensor S.
    Eigen::Vector3d v_Sk; ///< Velocity in frame Sk [m/s].
    Eigen::Vector3d omega_S_raw; ///< raw rotation rate in frame S, not compensated for bias.
    Time timestamp; ///< Timestamp corresponding to this state.
    StateId keyframeId; ///< Frame Id.
    StateId id; ///< Frame Id.
    ImuMeasurementDeque previousImuMeasurements; ///< IMU measurements up to this state's time.
  };

  /// \brief Stores the non-keyframe State information by State ID.
  std::map<StateId, AlignedMap<StateId, NonKeyframeState>> nonKeyframeStatesByKeyframeId_;

  /// \brief Associates the closest (earlier) state ID for a given timestamp.
  AlignedMap<uint64_t, StateId> statesByTimestampUs_;    
  
  std::mutex stateMutex_;
};

/**
 * @brief An abstract base class for interfaces between Front- and Backend.
 */
class ViInterface
{
 public:
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// \brief Unified callback function.
  typedef std::function<void(const State &, const TrackingState &,
                             std::shared_ptr<const AlignedMap<StateId, State>>,
                             std::shared_ptr<const okvis::MapPointVector>)> OptimisedGraphCallback;

  /// @brief Default constructor, not doing anything.
  ViInterface();

  /// @brief Destructor, closing file writing etc.
  virtual ~ViInterface();

  /// \name Setters
  /// \{

  /// \brief              Set a CVS file where the IMU data will be saved to.
  /// \param csvFile      The file.
  bool setImuCsvFile(std::fstream& csvFile);

  /// \brief              Set a CVS file where the IMU data will be saved to.
  /// \param csvFileName  The filename of a new file.
  bool setImuCsvFile(const std::string& csvFileName);

  /// \brief              Set a CVS file where the tracks (data associations) will be saved to.
  /// \param cameraId     The camera ID.
  /// \param csvFile      The file.
  bool setTracksCsvFile(size_t cameraId, std::fstream& csvFile);

  /// \brief              Set a CVS file where the tracks (data associations) will be saved to.
  /// \param cameraId     The camera ID.
  /// \param csvFileName  The filename of a new file.
  bool setTracksCsvFile(size_t cameraId, const std::string& csvFileName);


  /// \}
  /// \name Add measurements to the algorithm.
  /// \{
  /**
   * \brief          Add a set of new image.
   * \param stamp    The image timestamp.
   * \param images   The images.
   * \return         Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) = 0;

  /**
   * \brief          Add an IMU measurement.
   * \param stamp    The measurement timestamp.
   * \param alpha    The acceleration measured at this time.
   * \param omega    The angular velocity measured at this time.
   * \return Returns true normally. False if the previous one has not been processed yet.
   */
  virtual bool addImuMeasurement(const okvis::Time & stamp,
                                 const Eigen::Vector3d & alpha,
                                 const Eigen::Vector3d & omega) = 0;
  /// \}
  /// \name Setters
  /// \{

  /// \brief Set the callback to be called every time the graph has been optimised.
  ///        A list of all changed states is provided, which can be usefull to keep
  ///        track e.g. upon loop closures
  virtual void setOptimisedGraphCallback(const OptimisedGraphCallback & optimisedGraphCallback);

  /**
   * \brief Set the blocking variable that indicates whether the addMeasurement() functions
   *        should return immediately (blocking=false), or only when the processing is complete.
   */
  virtual void setBlocking(bool blocking) = 0;

  /// \brief Whether or not propagation with latest IMU measurements to be used (and published)
  /// \param realtimePropagation Whether or not to propagate.
  virtual void setRealtimePropagation(bool realtimePropagation);
  /// \}
  

  /// @brief Display some visualisation.
  virtual void display(cv::Mat & images, cv::Mat & topDebugImg) = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:

  /// \brief Write first line of IMU CSV file to describe columns.
  bool writeImuCsvDescription();

  /// \brief Write first line of tracks (data associations) CSV file to describe columns.
  bool writeTracksCsvDescription(size_t cameraId);

  OptimisedGraphCallback optimisedGraphCallback_; ///< Optimised graph callback function.

  std::shared_ptr<std::fstream> csvImuFile_;  ///< IMU CSV file.
  typedef std::map<size_t, std::shared_ptr<std::fstream>> FilePtrMap; ///< Map of file pointers.
  FilePtrMap csvTracksFiles_;  ///< Tracks CSV Files.

  std::atomic_bool realtimePropagation_; ///< Propagate to realtime?


};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_VIINTERFACE_HPP_ */
