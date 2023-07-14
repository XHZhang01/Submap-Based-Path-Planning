/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
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
 *  Created on: Aug 30, 2019
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file okvis/ViGraph.hpp
 * @brief Header file for the ViGraph2 class. This does all the backend work.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_VIGRAPH2_HPP_
#define INCLUDE_OKVIS_VIGRAPH2_HPP_

#include <memory>
#include <mutex>
#include <array>
#include <fstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <ceres/cost_function.h>
#include <ceres/crs_matrix.h>
#include <ceres/evaluation_callback.h>
#include <ceres/iteration_callback.h>
#include <ceres/loss_function.h>
#include <ceres/manifold.h>
#include <ceres/ordered_groups.h>
#include <ceres/problem.h>
#include <ceres/product_manifold.h>
#include <ceres/sized_cost_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <ceres/version.h>
#pragma GCC diagnostic pop

#include <okvis/kinematics/Transformation.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/RelativePoseError.hpp>
#include <okvis/ceres/TwoPoseGraphError.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/ceres/CeresIterationCallback.hpp>
#include <okvis/ceres/GpsErrorAsynchronous.hpp>
#include <okvis/ceres/SubmapIcpError.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <se/supereight.hpp>
#include <Eigen/StdVector>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief A class to construct visual-inertial optimisable graphs with.
class ViGraph
{
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class ViSlamBackend;

  /**
   * @brief The constructor.
   */
  ViGraph();

  /**
   * @brief The destructor (does nothing).
   */
  ~ViGraph() {}

  /**
   * @brief Add a camera to the configuration. Sensors can only be added and never removed.
   * @param cameraParameters The parameters that tell how to estimate extrinsics.
   * @return Index of new camera.
   */
  int addCamera(const okvis::CameraParameters & cameraParameters);

  /**
   * @brief Add an IMU to the configuration.
   * @warning Currently there is only one IMU supported.
   * @param imuParameters The IMU parameters.
   * @return index of IMU.
   */
  int addImu(const okvis::ImuParameters & imuParameters);

  /**
   * @brief Add a GPS sensor to the configuration.
   * @warning Currently there is only one GPS supported.
   * @param gpsParameters The GPS sensor parameters.
   * @return index of GPS.
   */
  int addGps(const okvis::GpsParameters & gpsParameters);

  // add states
  /**
   * @brief Add a new state and initialise position to zero translation and orientation with IMU.
   * @param timestamp The state's corresponding timestamp.
   * @param imuMeasurements IMU measurements to initialise orientation.
   * @param nCameraSystem The N-Camera system.
   * @return The state ID of the created state.
   */
  StateId addStatesInitialise(const Time& timestamp, const ImuMeasurementDeque & imuMeasurements,
                              const cameras::NCameraSystem & nCameraSystem);
  /**
   * @brief Add a new state by propagation of IMU.
   * @param timestamp The state's corresponding timestamp.
   * @param imuMeasurements IMU measurements to be used for propagation.
   * @param isKeyframe Add state as keyframe?
   * @return The state ID of the created state.
   */
  StateId addStatesPropagate(const Time& timestamp, const ImuMeasurementDeque & imuMeasurements,
                             bool isKeyframe);
  /**
   * @brief Add a new state from an other ViGraph object.
   * @param stateId The state ID to be used in other (and which will be created).
   * @param other The other graph.
   * otherwise the same parameter block pointer will be used (aliased).
   * @return True on success.
   */
  bool addStatesFromOther(StateId stateId, const ViGraph & other);

  // add/remove landmarks
  /**
   * @brief Add a new landmark with a defined ID.
   * @param landmarkId The landmark ID to be used.
   * @param homogeneousPoint The point position in World frame.
   * @param initialised Defines the landmark initialisation status (depth known or not).
   * @return True on success.
   */
  bool addLandmark(LandmarkId landmarkId, const Eigen::Vector4d &homogeneousPoint, bool initialised);
  /**
   * @brief Add a new landmark and get a new ID.
   * @param homogeneousPoint The point position in World frame.
   * @param initialised Defines the landmark initialisation status (depth known or not).
   * @return The ID of the newly created landmark.
   */
  LandmarkId addLandmark(const Eigen::Vector4d &homogeneousPoint, bool initialised);
  /**
   * @brief Remove landmark and get a new ID.
   * @param landmarkId The ID of the landmark to be removed.
   * @return True on successful removal.
   */
  bool removeLandmark(LandmarkId landmarkId);
  /**
   * @brief Set landmark initialisation.
   * @param landmarkId The ID of the landmark to be set.
   * @param initialised The initialisation status.
   * @return True on success.
   */
  bool setLandmarkInitialised(LandmarkId landmarkId, bool initialised);
  /**
   * @brief Set landmark quality.
   * @param landmarkId The ID of the landmark to be set.
   * @param quality The Landmark quality.
   * @return True on success.
   */
  bool setLandmarkQuality(LandmarkId landmarkId, double quality) {
    //OKVIS_CHECK_MAP(landmarks_,landmarkId);
    landmarks_.at(landmarkId).quality = quality;
    return true;
  }
  /**
   * @brief Get landmark initialisation.
   * @param landmarkId The ID of the landmark.
   * @return The initialisation status.
   */
  bool isLandmarkInitialised(LandmarkId landmarkId) const;
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param landmarkId The ID of the landmark.
   * @return The landmark position estimate (in World frame).
   */
  bool isLandmarkAdded(LandmarkId landmarkId) const;
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param id The ID of the landmark.
   * @return The landmark position estimate (in World frame).
   */
  const Eigen::Vector4d & landmark(LandmarkId id) const;
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param landmarkId The ID of the landmark.
   * @param mapPoint The landmark returned.
   * @return True on success.
   */
  bool getLandmark(LandmarkId landmarkId, okvis::MapPoint2& mapPoint) const;
  /**
   * @brief Get a copy of all the landmarks as a PointMap.
   * @param[out] landmarks The landmarks.
   * @return number of landmarks.
   */
  size_t getLandmarks(MapPoints & landmarks) const;
  /**
  * @brief Does the landmark exist?
  * @param landmarkId The ID of the landmark.
  * @return Whether the landmark exists.
  */
  bool landmarkExists(LandmarkId landmarkId) const;
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param id The ID of the landmark.
   * @param homogeneousPoint The landmark position estimate (in World frame).
   * @param initialised Whether the landmark is initialised.
   * @return True on success.
   */
  bool setLandmark(LandmarkId id, const Eigen::Vector4d & homogeneousPoint, bool initialised);
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param id The ID of the landmark.
   * @param homogeneousPoint The landmark position estimate (in World frame).
   * @return True on success.
   */
  bool setLandmark(LandmarkId id, const Eigen::Vector4d & homogeneousPoint);

  // add/remove observations
  /**
   * @brief Add observation.
   * @tparam GEOMETRY_TYPE Camera geometry type to use.
   * @param multiFrame The multiFrame containing the keypoint measurements to choose from.
   * @param landmarkId The ID of the landmark.
   * @param keypointId The keypoint ID {Multiframe ID, Camera index, Keypoint index}.
   * @param useCauchy Whether to use Cauchy robustification.
   * @return True on success.
   */
  template<class GEOMETRY_TYPE>
  bool addObservation(const MultiFrame& multiFrame, LandmarkId landmarkId,
                      KeypointIdentifier keypointId, bool useCauchy = true) {
    OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId), "landmark not added")

    // avoid double observations
    OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.at(landmarkId).observations.count(keypointId) == 0,
                          "observation already exists")
    OKVIS_ASSERT_TRUE_DBG(Exception, observations_.count(keypointId) == 0,
                          "observation already exists")
    OKVIS_ASSERT_TRUE_DBG(Exception, multiFrame.landmarkId(
                            keypointId.cameraIndex, keypointId.keypointIndex) == landmarkId.value(),
                          "observation already exists")

    // get the keypoint measurement
    Eigen::Vector2d measurement;
    multiFrame.getKeypoint(keypointId.cameraIndex, keypointId.keypointIndex, measurement);
    Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
    double size = 1.0;
    multiFrame.getKeypointSize(keypointId.cameraIndex, keypointId.keypointIndex, size);
    information *= 64.0 / (size * size);

    // create error term
    Observation observation;
    observation.errorTerm.reset(new ceres::ReprojectionError<GEOMETRY_TYPE>(
                    multiFrame.template geometryAs<GEOMETRY_TYPE>(keypointId.cameraIndex),
                    keypointId.cameraIndex, measurement, information));

    State& state = states_.at(StateId(keypointId.frameId));
    observation.residualBlockId = problem_->AddResidualBlock(
        observation.errorTerm.get(),
        useCauchy&&cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : nullptr,
        state.pose->parameters(), landmarks_.at(landmarkId).hPoint->parameters(),
        state.extrinsics.at(keypointId.cameraIndex)->parameters());
    observation.landmarkId = landmarkId;

    // remember everywhere
    observations_[keypointId] = observation;
    landmarks_.at(landmarkId).observations[keypointId] = observation;
    state.observations[keypointId] = observation;

    // covisibilities invalid
    covisibilitiesComputed_ = false;

    return true;
  }
  /**
   * @brief Add observation from a reprojection error term.
   * @param reprojectionError The external reprojection error.
   * @param landmarkId The ID of the landmark.
   * @param keypointId The keypoint ID {Multiframe ID, Camera index, Keypoint index}.
   * @param useCauchy Whether to use Cauchy robustification.
   * @return True on success.
   */
  bool addExternalObservation(
      const std::shared_ptr<const ceres::ReprojectionError2dBase> & reprojectionError,
       LandmarkId landmarkId, KeypointIdentifier keypointId, bool useCauchy = true) {
    OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId), "landmark not added")

    // avoid double observations
    OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.at(landmarkId).observations.count(keypointId) == 0,
                          "observation already exists")
    OKVIS_ASSERT_TRUE_DBG(Exception, observations_.count(keypointId) == 0,
                          "observation already exists")

    // create error term
    Observation observation;
    observation.errorTerm = reprojectionError->clone();

    State& state = states_.at(StateId(keypointId.frameId));
    observation.residualBlockId = problem_->AddResidualBlock(
        observation.errorTerm.get(),
        useCauchy&&cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : nullptr,
        state.pose->parameters(), landmarks_.at(landmarkId).hPoint->parameters(),
        state.extrinsics.at(keypointId.cameraIndex)->parameters());
    observation.landmarkId = landmarkId;

    // remember everywhere
    observations_[keypointId] = observation;
    landmarks_.at(landmarkId).observations[keypointId] = observation;
    state.observations[keypointId] = observation;

    // covisibilities invalid
    covisibilitiesComputed_ = false;

    return true;
  }
  /**
   * @brief Remove observation.
   * @param keypointId The keypoint ID {Multiframe ID, Camera index, Keypoint index}.
   * @return True on success.
   */
  bool removeObservation(KeypointIdentifier keypointId);

  /// \brief Computes the co-visibilities of all observed frames.
  /// \return True on success.
  bool computeCovisibilities();

  /// \brief Get the convisibilities between two frames.
  /// \note Need to call computeCovisiblities before!
  /// \param pose_i The one state.
  /// \param pose_j The other state.
  /// \return Number of covisible landmarks.
  int covisibilities(StateId pose_i, StateId pose_j) const;

  /// \brief Add a relative pose error between two poses in the graph.
  /// \param poseId0 ID of one pose.
  /// \param poseId1 ID of the other pose.
  /// \param T_S0S1 Relative transform (measurement).
  /// \param information Associated information matrix.
  /// \return True on success.
  bool addRelativePoseConstraint(StateId poseId0, StateId poseId1,
                                 const kinematics::Transformation &T_S0S1,
                                 const Eigen::Matrix<double, 6, 6>& information);
  /// \brief Remove a relative pose error between two poses in the graph.
  /// \param poseId0 ID of one pose.
  /// \param poseId1 ID of the other pose.
  /// \return True on success.
  bool removeRelativePoseConstraint(StateId poseId0, StateId poseId1);

  /// \brief Add a GPS measurement to one pose of the graph.
  /// \param poseId ID of the pose whose position is measured
  /// \param gpsMeas the GPS measurement to be added (position, covariance, timestamp information)
  /// \param imuMeasurements IMU measurements covering at least time span from state to GPS measurement
  /// \return ResidualBlockId of added residual block
  /*::ceres::ResidualBlockId*/ bool addGpsMeasurement(StateId poseId, GpsMeasurement &gpsMeas, const ImuMeasurementDeque &imuMeasurements);

  /// \brief Add multiple GPS measurements as factors on the graph.
  /// \param gpsMeasurementDeque Queue of GPS Measurements
  /// \param imuMeasurementDeque Queue of IMU Measurements
  /// \param[out] sids State IDs of states that the GPS measurements are assigned to
  /// \return True on success.
  bool addGpsMeasurements(GpsMeasurementDeque& gpsMeasurementDeque, ImuMeasurementDeque& imuMeasurementDeque, std::deque<StateId>* sids);

  /// \brief Freeze External GPS Trafo (T_GW)
  void freezeGpsExtrinsics();

  /// \brief Check if stateID exists in graph
  /// \return True if state with id exists. False otherwise
  bool findStateId(StateId sid){ return states_.count(sid);}

  /// \brief Check Status of GPS observability
  /// \return True if GPS Trafo is observable.
  bool isGpsObservable(){return gpsObservability_;}

  /// \brief Check if GPS trafo is fixed
  /// \return True if GPS Trafo is fixed.
  bool isGpsFixed(){return gpsFixed_;}

  /// \brief Check if GPS alignment is required
  /// \param[out] GPS alignment information, id of state when GPS signal is lost and IDs of states used during re-initialisation
  /// \return True if alignment required.
  bool needsGpsAlignment(StateId& gpsLossId, std::set<StateId>& reInitStates);

  /// \brief Check if full GPS alignment is required
  /// \param[out] GPS alignment information, id of state when GPS signal is lost and IDs of states used during re-initialisation
  /// \return True if full alignment required.
  bool needsFullGpsAlignment(StateId& gpsLossId, StateId& posAlignId, okvis::kinematics::Transformation& T_GW_new);

  /// \brief Check if position alignment is required
  /// \param[out]  GPS alignment information, id of state when GPS signal is lost and IDs of states used during re-initialisation
  /// \return True if position alignment required.
  bool needsPosGpsAlignment(StateId& gpsLossId, StateId& gpsAlignId, Eigen::Vector3d& posError);

  /// \brief Check if alignment is required after init stage
  /// \return True if alignment required.
  bool needsInitialGpsAlignment();

  /// \brief Check if GPS Re-Initialisation is needed
  /// \return True if re-initialisation is needed (last state freeze), false if not
  bool needsGpsReInit();

  /// \brief re-initialise GPS extrinsics
  void reInitGpsExtrinsics();

  /// \brief reset after full alignment
  void resetFullGpsAlignment();
  /// \brief reset after position alignment
  void resetPosGpsAlignment();
  /// \brief reset after initial alignment
  void resetInitialGpsAlignment(){
    gpsInitMap_.clear();
    gpsInitImuQueue_.clear();
    needsInitialAlignment_=false;
  }


  /// \brief Reset GPS Alignment parameters after alignment has been executed from backend
  void resetGpsAlignment();

  /**
   * @brief Set the extrinsic gps trafo (T_GW).
   * @param pose The pose (T_GW).
   * @return True on success.
   */
  bool setGpsExtrinsics(const kinematics::TransformationCacheless & T_GW);

  /**
   * @brief Writing CSV File containing GPS residuals. ATTENTION: Information set to Identity, so only call it after shutdown!
   * @param gpsResCsvFileName Name of the CSV file to be written
   */
  void dumpGpsResiduals(const std::string & gpsResCsvFileName){

      // Open File
      std::ofstream residualsOutput(gpsResCsvFileName);
      residualsOutput << "# tk , tg , res_x , res_y , res_z \n";

      // Retrieve Residual Blocks
      std::vector<::ceres::ResidualBlockId> residualIds;
      problem_->GetResidualBlocksForParameterBlock(states_.at(okvis::StateId(1)).T_GW->parameters(), &residualIds);

      std::cout << "[DEBUG INFO Summary] Dumping " << residualIds.size() << " residuals for " << states_.size() << " states." << std::endl;

      for(auto res :residualIds){
          double cost = 0.0;

          // Obtain Cost Function Object for every residual block associated with T_GW
          const ::ceres::CostFunction* costFct;
          costFct = problem_->GetCostFunctionForResidualBlock(res);


          static_cast<okvis::ceres::GpsErrorAsynchronous*>(const_cast<::ceres::CostFunction*>(costFct))->setInformation(Eigen::Matrix3d::Identity());

          okvis::Time tg;
          tg = static_cast<okvis::ceres::GpsErrorAsynchronous*>(const_cast<::ceres::CostFunction*>(costFct))->tg();
          okvis::Time tk;
          tk = static_cast<okvis::ceres::GpsErrorAsynchronous*>(const_cast<::ceres::CostFunction*>(costFct))->tk();

          Eigen::Matrix<double,3,1> residuals;
          problem_->EvaluateResidualBlock(res,false,&cost,residuals.data(),nullptr);

          residualsOutput << tk << " , " << tg << " , "
                          << residuals(0) << " , "
                          << residuals(1) << " , "
                          << residuals(2) << std::endl;

      }
      residualsOutput.close();

  }


  /**
   * @brief Obtain the Hessian block for a specific landmark.
   * @param[in] lmId Landmark ID of interest.
   * @param[out] H the output Hessian block.
   */
  void getLandmarkHessian(LandmarkId lmId, Eigen::Matrix3d& H);

  /**
   * @brief Compute observability of T_GW (covariance).
   * @param[in] covOptions Settings for computation of covariance.
   * @param[out] gpsObservability Parameters of covariance matrix of estimated T_GW.
   * @return True on success. False if computation fails (not observable / rank-deficient Jacobians in sparse case).
   */
  bool computeGpsObservability(::ceres::Covariance::Options& covOptions, double* gpsObservability);

  /// \brief Compute observability of extrinsic gps transformation (based on heuristics)
  /// \return True if observable, false if non-observable.
  bool computeGpsObservability(/*double threshold*/ /* moved into parameter struct*/);
  /// \brief Compute observability of extrinsic gps transformation (based on heuristics)
  /// \param Parameter Block (GPS Measurements) for which the Observability is computed
  /// \return True if observable, false if non-observable.
  bool computeGpsObservability(okvis::ceres::PoseParameterBlock& gpsExtrinsics );

  /// \brief Initialise T_GW from measurements using SVD
  /// \param T_GW: Transformation to write initialisation to
  /// \return
  void initialiseGpsExtrinsics(okvis::kinematics::Transformation& T_GW);

  /// \brief fct for debugging
  /// \param T_GW: Transformation to write initialisation to
  bool checkForGpsInit(okvis::kinematics::Transformation& T_GW, std::set<StateId> consideredStates);

  /// \brief             Add Alignment constraints from submapping interface
  /// @param frame_A_id  ID of frame {A}
  /// @param frame_B_id  ID of frame {B}
  /// @param pointCloud  Point Cloud in {B} that adds constraints w.r.t. {B}
  /// \return Returns true normally
  bool addSubmapAlignmentConstraints(const se::OccupancyMap<se::Res::Multi>* submap_ptr,
                                     const uint64_t& frame_A_id, const uint64_t frame_B_id,
                                     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& pointCloud);

  // get/set states
  /**
   * @brief Get the pose (T_WS).
   * @param id The state ID from which go get the pose.
   * @return The pose (T_WS).
   */
  const kinematics::TransformationCacheless & pose(StateId id) const;
  /**
   * @brief Get the speed/biases [v_W, b_g, b_a].
   * @param id The state ID from which go get the speed/biases.
   * @return The pose [v_W, b_g, b_a].
   */
  const SpeedAndBias & speedAndBias(StateId id) const;
  /**
   * @brief Get the extrinsics pose (T_SC).
   * @param id The state ID from which go get the extrinsics.
   * @param camIdx The camera index of the extrinsics.
   * @return The extrinsics pose (T_SC).
   */
  const kinematics::TransformationCacheless & extrinsics(StateId id, uchar camIdx) const;

  /**
   * @brief Get the GPS <-> World Trafo (T_GW)
   * @return The trafo (T_GW).
   */
  const kinematics::TransformationCacheless & T_GW() const;

  /**
   * @brief Get the GPS <-> World Trafo (T_GW) for specific state
   * @return The trafo (T_GW).
   */
  const kinematics::TransformationCacheless & T_GW(StateId id) const;

  /// \brief Is it a keyframe?
  /// \brief id The state ID in question.
  /// \return True if it is.
  bool isKeyframe(StateId id) const;

  /// \brief Get the timestamp.
  /// \brief id The state ID in question.
  /// \return The timestamp.
  Time timestamp(StateId id) const;

  /// \brief Find the most recent frame added.
  /// \return ID of most recent frame.
  StateId currentStateId() const;

  /// \brief Find an older frame added.
  /// \param age How far back to go.
  /// \return ID of most recent frame.
  StateId stateIdByAge(size_t age) const;

  /// \brief Make a frame a keyframe.
  /// \warning This can only be done for IMU frames.
  /// \param id The state ID in question.
  /// \param isKeyframe Whether or not it should be a keyframe.
  /// \return True on success.
  bool setKeyframe(StateId id, bool isKeyframe);

  /**
   * @brief Set the pose (T_WS).
   * @param id The state ID for which go set the pose.
   * @param pose The pose (T_WS).
   * @return True on success.
   */
  bool setPose(StateId id, const kinematics::TransformationCacheless & pose);
  /**
   * @brief Set the speed/biases [v_W, b_g, b_a].
   * @param id The state ID for which go set the speed/biases.
   * @param speedAndBias The the speed/biases [v_W, b_g, b_a].
   * @return True on success.
   */
  bool setSpeedAndBias(StateId id, const SpeedAndBias & speedAndBias);
  /**
   * @brief Set the extrinsics pose (T_SC).
   * @param id The state ID for which go set the extrinsics.
   * @param camIdx The camera index of the extrinsics.
   * @param extrinsics The extrinsics pose (T_SC).
   * @return True on success.
   */
  bool setExtrinsics(
      StateId id, uchar camIdx, const kinematics::TransformationCacheless & extrinsics) const;

  /// \brief Sets all extrinsics to be optimised.
  /// \return True on success.
  bool setExtrinsicsVariable();

  /// \brief Gives all extrinsice a pose prior.
  /// \param posStd Position uncertainty standard deviation.
  /// \param rotStd Orientation uncertainty standard deviation.
  /// \return True on success.
  bool softConstrainExtrinsics(double posStd, double rotStd);

  // get/set ceres stuff
  /**
   * @brief Get the ceres optimisation options.
   * @return The ceres optimisation options.
   */
  const ::ceres::Solver::Options & options() const { return options_; }
  /**
   * @brief Get the ceres optimisation options (modifiable).
   * @return The ceres optimisation options (modifiable).
   */
  ::ceres::Solver::Options & options() { return options_; }
  /**
   * @brief Get the ceres optimisation summary.
   * @return The ceres optimisation summary.
   */
  const ::ceres::Solver::Summary & summary() const { return summary_; }

  /**
   * @brief Solve the optimisation problem.
   * @param[in] maxIterations Maximum number of iterations.
   * @param[in] numThreads Number of threads.
   * @param[in] verbose Print out optimisation progress and result, if true.
   */
  void optimise(int maxIterations, int numThreads, bool verbose);

  /// \brief Set a limit for realtime-ish operation.
  /// \param timeLimit Maximum time allowed [s].
  /// \param minIterations Minimum iterations to be carried out irrespective of time limit.
  /// \return True on success.
  bool setOptimisationTimeLimit(double timeLimit, int minIterations);

  /// \brief Removes landmarks that are not observed.
  /// \return The number of landmarks removed.
  int cleanUnobservedLandmarks(
      std::map<LandmarkId, std::set<KeypointIdentifier>> *removed = nullptr);

  /// \brief Update landmark quality and initialisation status using current graph/estimates.
  void updateLandmarks();

protected:

  /// \brief Check observation consistency.
  void checkObservations() const;

  /// \brief Helper struct to store specific edges.
  template<typename ErrorTermT>
  struct GraphEdge {
    ::ceres::ResidualBlockId residualBlockId = nullptr; ///< Ceres residual pointer.
    std::shared_ptr<ErrorTermT> errorTerm; ///< Error term.
  };

  /// \brief Helper struct for generic binary graph edges between poses.
  template<typename ErrorTermT>
  struct TwoStateGraphEdge : public GraphEdge<ErrorTermT>{
    StateId state0; ///< Reference state ID.
    StateId state1; ///< Other state ID.
  };

  /// \brief Helper struct for the reprojection error graph edges.
  struct Observation : public GraphEdge<ceres::ReprojectionError2dBase> {
    LandmarkId landmarkId; ///< Landmark ID.
  };

  /// \brief Helper struct for the IMU error graph edges.
  using ImuLink = GraphEdge<ceres::ImuError>;

  /// \brief Helper struct for the extrinsics pose change binary edges.
  using ExtrinsicsLink = GraphEdge<ceres::RelativePoseError>;

  /// \brief Helper struct for pose error unary edges.
  using PosePrior = GraphEdge<ceres::PoseError>;

  /// \brief Helper struct for speed and bias error unary edges.
  using SpeedAndBiasPrior = GraphEdge<ceres::SpeedAndBiasError>;

  /// \brief Helper struct for extrinsics pose error unary edges.
  using ExtrinsicsPrior = GraphEdge<ceres::PoseError>;

  /// \brief Binary pose graph edge.
  using TwoPoseLink = TwoStateGraphEdge<ceres::TwoPoseGraphError>;

  /// \brief Binary pose graph edge (const version, i.e. not convertible to/from observations).
  using TwoPoseConstLink = TwoStateGraphEdge<ceres::TwoPoseGraphErrorConst>;

  /// \brief Relative pose graph edge.
  using RelativePoseLink = TwoStateGraphEdge<ceres::RelativePoseError>;

  /// \brief GPS factpr pose graph edge.
  using GpsFactor = GraphEdge<ceres::GpsErrorAsynchronous>;

  /// \brief
  using SubmapAlignmentFactor = GraphEdge<ceres::SubmapIcpError>;



  /// \brief Extended state info (including graph edges)
  struct State {
    // these are holding the estimates underneath
    std::shared_ptr<ceres::PoseParameterBlock> pose; ///< Pose parameter block.
    std::shared_ptr<ceres::SpeedAndBiasParameterBlock> speedAndBias; ///< Speed/bias param. block.
    std::vector<std::shared_ptr<ceres::PoseParameterBlock>> extrinsics; ///< Extinsics param. block.
    std::shared_ptr<ceres::PoseParameterBlock> T_GW; ///< world to global transform param. block.

    // error terms
    std::map<KeypointIdentifier, Observation> observations; ///< All observations per keypoint.
    ImuLink nextImuLink; ///< IMU link to next state.
    ImuLink previousImuLink; ///< IMU link to next previous.
    std::vector<ExtrinsicsLink> nextExtrinsicsLink; ///< Link to next extrinsics.
    std::vector<ExtrinsicsLink> previousExtrinsicsLink; ///< Link to previous extrinsics.
    PosePrior posePrior; ///< Pose prior.
    SpeedAndBiasPrior speedAndBiasPrior; ///< Speed/bias prior.
    std::vector<ExtrinsicsPrior> extrinsicsPriors; ///< Extrinsics prior.
    std::map<StateId, TwoPoseLink> twoPoseLinks; ///< All pose graph edges.
    std::map<StateId, TwoPoseConstLink> twoPoseConstLinks; ///< All pose graph edges (const).
    std::map<StateId, RelativePoseLink> relativePoseLinks; ///< All relative pose graph edges.
    std::vector<GpsFactor> GpsFactors; ///< All GPS factors
    // ToDo: how to store  submap alignment factors for two states
    //std::vector<SubmapAlignmentFactor> SubmapAlignmentFactors; ///< All Submap Alignment factors
    size_t gpsMode; ///< Status of GPS (Re-)Initialization.

    // attributes
    bool isKeyframe = false; ///< Is it a keyframe?
    okvis::Time timestamp = okvis::Time(0.0); ///< The timestamp.
  };

  /// \brief Landmark helper struct.
  struct Landmark {
    std::shared_ptr<ceres::HomogeneousPointParameterBlock> hPoint; ///< Point in world coordinates.
    std::map<KeypointIdentifier, Observation> observations; ///< All observations of it.
    double quality = 0.0; ///< 3D quality.
    int classification = -1; ///< It's classification (if used / classified by the CNN already).
  };


  // parameters
  std::vector<okvis::CameraParameters,
      Eigen::aligned_allocator<okvis::CameraParameters> > cameraParametersVec_; ///< Extrinsics.
  std::vector<okvis::ImuParameters,
      Eigen::aligned_allocator<okvis::ImuParameters> > imuParametersVec_; ///< IMU parameters.
  std::vector<okvis::GpsParameters,
      Eigen::aligned_allocator<okvis::GpsParameters> > gpsParametersVec_; ///< GPS parameters

  // this stores the elements of the graph (note the redundancy for spee in the states)
  std::map<StateId, State> states_; ///< Store all states.
  std::map<LandmarkId, Landmark> landmarks_; ///< Contains all current landmarks.
  std::map<KeypointIdentifier, Observation> observations_; ///< Contains all observations.

  /// GPS (Re-)Initialization Status
  enum gpsStatus{
      Off = 0,
      Idle = 99,
      Initialising = 1,
      Initialised = 2,
      ReInitialising = 3
  };

  gpsStatus gpsStatus_ = gpsStatus::Off; /// < Indicator what status of GPS reception we are in
  bool gpsObservability_ = false; /// < Flag if T_GW is observable with so-far measurements; init with false since GPS extrinsics not observable before first measurements are added
  bool gpsFixed_ = false; /// < Flag if gps parameter block is currently fixed

  std::set<StateId> gpsStates_; /// < Set containing IDs of states connected to global position factors
  std::multimap<StateId, GpsMeasurement> gpsInitMap_;
  ImuMeasurementDeque gpsInitImuQueue_; /// < Queue buffering IMU measurements during initialisation

  // Re-initialisation and alignment
  bool needsInitialAlignment_ = false; /// < Flag if full alignmemt (orientation + position) should be triggered
  bool needsPositionAlignment_ = false; /// < Flag if receiving GPS after long droput triggers position alignment
  bool needsFullAlignment_ = false; /// < Flag if full alignmemt (orientation + position) should be triggered

  StateId gpsDropoutId_; /// < Id of state with last gps measurement before break
  StateId positionAlignedId_; /// < ID of state which was only position aligned

  std::set<StateId> gpsReInitStates_; /// < Set containing States with gps measurements during re-initialization
  bool gpsReInitialised_ = false; /// < Flag if Re-Initialisation is successful and GPS LC can be triggered


  /// \brief Store 4D local parametrisation (position, yaw) for GPS extrinsics locally
  okvis::ceres::PoseManifold4d gpsExtrinsicLocalParametrisation_;

  /// \brief Store parameterisation locally.
  okvis::ceres::HomogeneousPointManifold homogeneousPointManifold_;

  /// \brief Store parameterisation locally.
  okvis::ceres::PoseManifold poseManifold_;

  /// \brief The ceres problem
  std::shared_ptr< ::ceres::Problem> problem_;

  /// \brief Ceres options
  ::ceres::Solver::Options options_;

  /// \brief Ceres optimization summary
  ::ceres::Solver::Summary summary_;

  // loss function for reprojection errors
  std::shared_ptr< ::ceres::LossFunction> cauchyLossFunctionPtr_; ///< Cauchy loss.
  std::shared_ptr< ::ceres::LossFunction> huberLossFunctionPtr_; ///< Huber loss.

  // ceres iteration callback object
  std::unique_ptr<ceres::CeresIterationCallback> ceresCallback_; ///< If callback registered, store.

  std::map<uint64_t, std::map<uint64_t, int>> coObservationCounts_; ///< Covisibilities cached.

  /// \brief If computeCovisibilities was called.
  /// Init with true since no observations in the beginning.
  bool covisibilitiesComputed_ = true;
  std::set<StateId> visibleFrames_; ///< Currently visible frames.

  /// \brief Helper struct for any state (i.e. keyframe or not).
  struct AnyState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StateId keyframeId; ///< Reference keyframe.
    Time timestamp; ///< The time.
    kinematics::Transformation T_Sk_S; ///< Transformation to keyframe.
    Eigen::Vector3d v_Sk; ///< Speed in keyframe coordinates
  };
  AlignedMap<StateId, AnyState> anyState_; ///< All states (including non-keyframes).

  // local cartesian frame from geodetic coordinates
  const GeographicLib::Geocentric& earth_ = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian globCartesianFrame_;
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_VIGRAPH_HPP_ */
