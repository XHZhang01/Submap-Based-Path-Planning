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
 * @file ViGraph.cpp
 * @brief Source file for the ViGraph class.
 * @author Stefan Leutenegger
 */

#include <okvis/ViGraph.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/timing/Timer.hpp>
#include "ceres/covariance.h"
#include <cmath>
#include <chrono>
#include <Eigen/SVD>
//#include <okvis/solveQuarticPolynom.hpp>
//#include <boost/math/distributions/chi_squared.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Add a camera to the configuration. Sensors can only be added and never removed.
ViGraph::ViGraph() : globCartesianFrame_(earth_)
{
  cauchyLossFunctionPtr_.reset(new ::ceres::CauchyLoss(1.0));
  ::ceres::Problem::Options problemOptions;
  problemOptions.manifold_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.loss_function_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.cost_function_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problem_.reset(new ::ceres::Problem(problemOptions));
  options_.linear_solver_type = ::ceres::SPARSE_NORMAL_CHOLESKY;
  options_.trust_region_strategy_type = ::ceres::DOGLEG;
  //options_.dense_linear_algebra_library_type = ::ceres::LAPACK; // somehow slow on Jetson...
}

int ViGraph::addCamera(const CameraParameters& cameraParameters) {
  cameraParametersVec_.push_back(cameraParameters);
  return static_cast<int>(cameraParametersVec_.size()) - 1;
}

// Add an IMU to the configuration.
int ViGraph::addImu(const ImuParameters& imuParameters) {
  if (imuParametersVec_.size() > 1) {
    LOG(ERROR) << "only one IMU currently supported";
    return -1;
  }
  imuParametersVec_.push_back(imuParameters);
  return static_cast<int>(imuParametersVec_.size()) - 1;
}

// Add a GPS sensor to the configuration.
int ViGraph::addGps(const GpsParameters& gpsParameters) {
  if (gpsParametersVec_.size() > 1) {
    LOG(ERROR) << "only one GPS currently supported";
    return -1;
  }
  gpsParametersVec_.push_back(gpsParameters);
  return static_cast<int>(gpsParametersVec_.size()) - 1;
}

StateId ViGraph::addStatesInitialise(
    const Time &timestamp, const ImuMeasurementDeque &imuMeasurements,
    const cameras::NCameraSystem & nCameraSystem)
{
  State state;
  state.timestamp = timestamp;
  state.isKeyframe = true; // first one must be keyframe.
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.size()==0, "states added before...")
  StateId id(1);

  // set translation to zero, unit rotation
  kinematics::Transformation T_WS;
  T_WS.setIdentity();

  OKVIS_ASSERT_TRUE_DBG(Exception, imuMeasurements.size() > 0, "no IMU measurements passed")

  // acceleration vector
  Eigen::Vector3d acc_B = Eigen::Vector3d::Zero();
  for (okvis::ImuMeasurementDeque::const_iterator it = imuMeasurements.begin();
       it < imuMeasurements.end(); ++it) {
    acc_B += it->measurement.accelerometers;
  }
  acc_B /= double(imuMeasurements.size());
  Eigen::Vector3d e_acc = acc_B.normalized();

  // align with ez_W:
  Eigen::Vector3d ez_W(0.0, 0.0, 1.0);
  Eigen::Matrix<double, 6, 1> poseIncrement;
  poseIncrement.head<3>() = Eigen::Vector3d::Zero();
  poseIncrement.tail<3>() = ez_W.cross(e_acc).normalized();
  double angle = std::acos(ez_W.transpose() * e_acc);
  poseIncrement.tail<3>() *= angle;
  T_WS.oplus(-poseIncrement);

  // now set/add states
  state.pose.reset(new ceres::PoseParameterBlock(T_WS, id.value(), timestamp));
  SpeedAndBias speedAndBias = SpeedAndBias::Zero();
  speedAndBias.tail<3>() = imuParametersVec_.at(0).a0;
  speedAndBias.segment<3>(3) = imuParametersVec_.at(0).g0;
  state.speedAndBias.reset(
        new ceres::SpeedAndBiasParameterBlock(speedAndBias, id.value(), timestamp));
  problem_->AddParameterBlock(state.pose->parameters(), 7, &poseManifold_);
  state.pose->setLocalParameterizationPtr(&poseManifold_);
  problem_->AddParameterBlock(state.speedAndBias->parameters(), 9);
  for(size_t i = 0; i<cameraParametersVec_.size(); ++i) {
    const kinematics::Transformation T_SC = *nCameraSystem.T_SC(i);
    state.extrinsics.push_back(std::shared_ptr<ceres::PoseParameterBlock>(
                                 new ceres::PoseParameterBlock(T_SC, id.value(), timestamp)));
    problem_->AddParameterBlock(state.extrinsics.back()->parameters(), 7,
                                &poseManifold_);
    state.extrinsics.back()->setLocalParameterizationPtr(&poseManifold_);
  }

  // set gps parameter block first time
  kinematics::Transformation T_GW0;
  T_GW0.setIdentity();
  state.T_GW.reset(new ceres::PoseParameterBlock(T_GW0,id.value(),timestamp));
  problem_->AddParameterBlock(state.T_GW->parameters(),7,&gpsExtrinsicLocalParametrisation_);
  state.T_GW->setLocalParameterizationPtr(&gpsExtrinsicLocalParametrisation_);
  problem_->SetParameterBlockVariable(state.T_GW->parameters());

  // add the priors
  Eigen::Matrix<double, 6, 1> informationDiag = Eigen::Matrix<double, 6, 1>::Ones();
  informationDiag[0] = 1.0e8;
  informationDiag[1] = 1.0e8;
  informationDiag[2] = 1.0e8;
  informationDiag[3] = 0.0;
  informationDiag[4] = 0.0;
  informationDiag[5] = 1.0e2;
  state.posePrior.errorTerm.reset(new ceres::PoseError(T_WS, informationDiag));
  const double sigma_bg = imuParametersVec_.at(0).sigma_bg;
  const double sigma_ba = imuParametersVec_.at(0).sigma_ba;
  state.speedAndBiasPrior.errorTerm.reset(
        new ceres::SpeedAndBiasError(speedAndBias, 0.1, sigma_bg * sigma_bg, sigma_ba * sigma_ba));
  state.posePrior.residualBlockId = problem_->AddResidualBlock(
        state.posePrior.errorTerm.get(), nullptr, state.pose->parameters());
  state.speedAndBiasPrior.residualBlockId = problem_->AddResidualBlock(
        state.speedAndBiasPrior.errorTerm.get(), nullptr, state.speedAndBias->parameters());
  for(size_t i = 0; i<cameraParametersVec_.size(); ++i) {
    if(cameraParametersVec_.at(i).online_calibration.do_extrinsics) {
      // add a pose prior
      PosePrior extrinsicsPrior;
      const double sigma_r = cameraParametersVec_.at(i).online_calibration.sigma_r;
      const double sigma_alpha = cameraParametersVec_.at(i).online_calibration.sigma_alpha;
      extrinsicsPrior.errorTerm.reset(
            new ceres::PoseError(
              state.extrinsics.at(i)->estimate(), sigma_r*sigma_r, sigma_alpha*sigma_alpha));
      extrinsicsPrior.residualBlockId = problem_->AddResidualBlock(
                extrinsicsPrior.errorTerm.get(), nullptr, state.extrinsics.at(i)->parameters());
      state.extrinsicsPriors.push_back(extrinsicsPrior);
    } else {
      // simply fix
      problem_->SetParameterBlockConstant(state.extrinsics.at(i)->parameters());
      state.extrinsics.at(i)->setFixed(true);
    }
  }

  states_[id] = state; // actually add...
  AnyState anyState;
  anyState.timestamp = state.timestamp;
  anyState.T_Sk_S = kinematics::Transformation::Identity();
  anyState.v_Sk = Eigen::Vector3d::Zero();
  anyState_[id] = anyState; // add, never remove.

  return id;
}

StateId ViGraph::addStatesPropagate(const Time &timestamp,
                                     const ImuMeasurementDeque &imuMeasurements, bool isKeyframe)
{
  // propagate state
  State state;
  state.timestamp = timestamp;
  state.isKeyframe = isKeyframe;
  StateId id(states_.rbegin()->first+1);
  State & lastState = states_.rbegin()->second;
  kinematics::Transformation T_WS = lastState.pose->estimate();
  SpeedAndBias speedAndBias = lastState.speedAndBias->estimate();
  ceres::ImuError::propagation(imuMeasurements, imuParametersVec_.at(0), T_WS, speedAndBias,
                               lastState.timestamp, timestamp);
  state.pose.reset(new ceres::PoseParameterBlock(T_WS, id.value(), timestamp));
  problem_->AddParameterBlock(state.pose->parameters(), 7, &poseManifold_);
  state.pose->setLocalParameterizationPtr(&poseManifold_);
  state.speedAndBias.reset(new ceres::SpeedAndBiasParameterBlock(
                             speedAndBias, id.value(), timestamp));
  problem_->AddParameterBlock(state.speedAndBias->parameters(), 9);

  // create IMU link
  ImuLink imuLink;
  imuLink.errorTerm.reset(new ceres::ImuError(imuMeasurements, imuParametersVec_.at(0),
                                              lastState.timestamp, timestamp));
  imuLink.residualBlockId = problem_->AddResidualBlock(
        imuLink.errorTerm.get(), nullptr,
        lastState.pose->parameters(), lastState.speedAndBias->parameters(),
        state.pose->parameters(), state.speedAndBias->parameters());

  // store IMU link
  lastState.nextImuLink = imuLink;
  state.previousImuLink = imuLink;

  // propagate extrinsics (if needed)
  for(size_t i = 0; i<cameraParametersVec_.size(); ++i) {
    // re-use same extrinsics
    state.extrinsics.push_back(lastState.extrinsics.at(i));
  }

  // GPS trafo: point back to initial parameter blocj
  state.T_GW = lastState.T_GW;
  state.GpsFactors.clear();

  states_[id] = state; // actually add...
  AnyState anyState;
  anyState.timestamp = state.timestamp;
  anyState.T_Sk_S = kinematics::Transformation::Identity();
  anyState.v_Sk = Eigen::Vector3d::Zero();
  anyState_[id] = anyState; // add, never remove.

  return id;
}

bool ViGraph::addStatesFromOther(StateId stateId, const ViGraph &other)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, other.states_.count(stateId), "stateId not found")

  State state;
  const State& otherState = other.states_.at(stateId);
  state.timestamp = otherState.timestamp;
  state.isKeyframe = otherState.isKeyframe;

  // clone states
  const size_t numCameras = otherState.extrinsics.size();
  state.extrinsics.resize(numCameras);

  // create all new from otherState
  state.pose.reset(new ceres::PoseParameterBlock(
                     otherState.pose->estimate(), stateId.value(), otherState.timestamp));
  state.speedAndBias.reset(new ceres::SpeedAndBiasParameterBlock(
                     otherState.speedAndBias->estimate(), stateId.value(), otherState.timestamp));
  for(size_t i = 0; i < numCameras; ++i) {
    state.extrinsics.at(i).reset(
            new ceres::PoseParameterBlock(otherState.extrinsics.at(i)->estimate(),
                                          stateId.value(), otherState.timestamp));
  }
  // GPS trafo; Trafo stays the same!!
  state.T_GW = otherState.T_GW;


  // handle priors, if any
  if(otherState.posePrior.errorTerm) {
    PosePrior& posePrior = state.posePrior;
    posePrior.errorTerm.reset(new ceres::PoseError(otherState.posePrior.errorTerm->measurement(),
                                                   otherState.posePrior.errorTerm->information()));
    posePrior.residualBlockId = problem_->AddResidualBlock(posePrior.errorTerm.get(),
                                                           nullptr, state.pose->parameters());
  }
  if(otherState.speedAndBiasPrior.errorTerm) {
    SpeedAndBiasPrior& speedAndBiasPrior = state.speedAndBiasPrior;
    speedAndBiasPrior.errorTerm.reset(
          new ceres::SpeedAndBiasError(otherState.speedAndBiasPrior.errorTerm->measurement(),
                                       otherState.speedAndBiasPrior.errorTerm->information()));
    speedAndBiasPrior.residualBlockId = problem_->AddResidualBlock(
          speedAndBiasPrior.errorTerm.get(), nullptr, state.speedAndBias->parameters());
  }
  for(size_t i=0; i<otherState.extrinsicsPriors.size(); ++i) {
    PosePrior extrinsicsPrior;
    const PosePrior& otherExtrinsicsPrior = state.extrinsicsPriors.at(i);
    extrinsicsPrior.errorTerm.reset(
          new ceres::PoseError(otherExtrinsicsPrior.errorTerm->measurement(),
                               otherExtrinsicsPrior.errorTerm->information()));
    extrinsicsPrior.residualBlockId = problem_->AddResidualBlock(
          extrinsicsPrior.errorTerm.get(), nullptr, state.extrinsics.at(i)->parameters());
    state.extrinsicsPriors.push_back(extrinsicsPrior);
  }

  // handle links, if any
  if(otherState.previousImuLink.errorTerm) {
    auto otherIter = other.states_.find(stateId);
    OKVIS_ASSERT_TRUE_DBG(Exception, otherIter != other.states_.begin(), "no previous state")
    otherIter--;
    const State& otherPreviousState = otherIter->second;
    const Time t_0 = otherPreviousState.timestamp;
    const Time t_1 = otherState.timestamp;
    State& previousState = states_.rbegin()->second;
    OKVIS_ASSERT_TRUE_DBG(Exception, otherIter->first == states_.rbegin()->first,
                      "different previous states")
    OKVIS_ASSERT_TRUE_DBG(Exception, t_0 == previousState.timestamp, "inconsistent previous times")
    ImuLink imuLink;
    imuLink.errorTerm = otherState.previousImuLink.errorTerm->clone();
    imuLink.residualBlockId = problem_->AddResidualBlock(
          imuLink.errorTerm.get(), nullptr,
          previousState.pose->parameters(), previousState.speedAndBias->parameters(),
          state.pose->parameters(), state.speedAndBias->parameters());
    state.previousImuLink = imuLink;
    previousState.nextImuLink = imuLink;

    // and possibly relative camera poses
    for(size_t i=0; i<otherState.previousExtrinsicsLink.size(); ++i) {
      ExtrinsicsLink extrinsicsLink;
      const ceres::RelativePoseError& otherRelativePoseError =
          *otherState.previousExtrinsicsLink.at(i).errorTerm;
      extrinsicsLink.errorTerm.reset(
            new ceres::RelativePoseError(otherRelativePoseError.information()));
      extrinsicsLink.residualBlockId = problem_->AddResidualBlock(
            extrinsicsLink.errorTerm.get(), nullptr, previousState.extrinsics.at(i)->parameters(),
            state.extrinsics.at(i)->parameters());
      state.previousExtrinsicsLink.push_back(extrinsicsLink);
      previousState.nextExtrinsicsLink.push_back(extrinsicsLink);
    }
  }

  states_[stateId] = state; // actually add...
  AnyState anyState;
  anyState.timestamp = state.timestamp;
  anyState.T_Sk_S = kinematics::Transformation::Identity();
  anyState.v_Sk = Eigen::Vector3d::Zero();
  anyState_[stateId] = anyState; // add, never remove.

  return true;
}

bool ViGraph::addLandmark(LandmarkId landmarkId, const Eigen::Vector4d &homogeneousPoint,
                           bool initialised)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarkId.isInitialised(), "landmark ID invalid")
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId) == 0, "landmark already exists")
  Landmark landmark;
  landmark.hPoint.reset(new ceres::HomogeneousPointParameterBlock(
                          homogeneousPoint, landmarkId.value(), initialised));
  problem_->AddParameterBlock(landmark.hPoint->parameters(), 4,
                              &homogeneousPointManifold_);
  landmark.hPoint->setLocalParameterizationPtr(&homogeneousPointManifold_);
  landmarks_[landmarkId] = landmark;
  return true;
}

LandmarkId ViGraph::addLandmark(const Eigen::Vector4d &homogeneousPoint, bool initialised)
{
  const LandmarkId landmarkId = landmarks_.empty() ?
        LandmarkId(1) : LandmarkId(landmarks_.rbegin()->first+1); // always increase highest ID by 1
  Landmark landmark;
  landmark.hPoint.reset(new ceres::HomogeneousPointParameterBlock(
                          homogeneousPoint, landmarkId.value(), initialised));
  problem_->AddParameterBlock(landmark.hPoint->parameters(), 4,
                              &homogeneousPointManifold_);
  landmark.hPoint->setLocalParameterizationPtr(&homogeneousPointManifold_);
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId) == 0, "terrible bug")
  landmarks_[landmarkId] = landmark;
  return landmarkId;
}

bool ViGraph::removeLandmark(LandmarkId landmarkId)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId), "landmark does not exists")
  Landmark& landmark = landmarks_.at(landmarkId);

  // remove all observations
  for(auto observation : landmark.observations) {
    problem_->RemoveResidualBlock(observation.second.residualBlockId);
    observations_.erase(observation.first);
    // also remove it in the state
    StateId stateId(observation.first.frameId);
    states_.at(stateId).observations.erase(observation.first);
  }

  // now remove the landmark itself
  problem_->RemoveParameterBlock(landmark.hPoint->parameters());
  landmarks_.erase(landmarkId);
  return true;
}

bool ViGraph::setLandmarkInitialised(LandmarkId landmarkId, bool initialised)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId), "landmark does not exists")
  landmarks_.at(landmarkId).hPoint->setInitialized(initialised);
  return true;
}

bool ViGraph::isLandmarkInitialised(LandmarkId landmarkId) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId), "landmark does not exists")
  return landmarks_.at(landmarkId).hPoint->initialized();
}

bool ViGraph::isLandmarkAdded(LandmarkId landmarkId) const
{
  return landmarks_.count(landmarkId)>0;
}

void ViGraph::checkObservations() const {

  // check by overall observations
  for(auto obs : observations_) {
    OKVIS_ASSERT_TRUE(
          Exception, landmarks_.at(obs.second.landmarkId).observations.count(obs.first), "bad")
    OKVIS_ASSERT_TRUE(
          Exception, states_.at(StateId(obs.first.frameId)).observations.count(obs.first), "bad")
  }

  // check by states
  for(auto state : states_) {
    for(auto obs : state.second.observations) {
      OKVIS_ASSERT_TRUE(
            Exception, landmarks_.at(obs.second.landmarkId).observations.count(obs.first), "bad")
      OKVIS_ASSERT_TRUE(Exception, observations_.count(obs.first), "bad")
    }
  }

  // check by landmarks
  for(auto lm : landmarks_) {
    for(auto obs : lm.second.observations) {
      OKVIS_ASSERT_TRUE(Exception, observations_.count(obs.first), "bad")
      OKVIS_ASSERT_TRUE(
            Exception, states_.at(StateId(obs.first.frameId)).observations.count(obs.first), "bad")
    }
  }

}

bool ViGraph::removeObservation(KeypointIdentifier keypointId)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, observations_.count(keypointId), "observation does not exists")
  Observation observation = observations_.at(keypointId);

  // remove in ceres
  problem_->RemoveResidualBlock(observation.residualBlockId);

  // remove everywhere in bookkeeping
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(observation.landmarkId),
                        "landmark does not exists")
  Landmark& landmark = landmarks_.at(observation.landmarkId);
  OKVIS_ASSERT_TRUE_DBG(Exception, landmark.observations.count(keypointId),
                    "observation does not exists")
  landmark.observations.erase(keypointId);
  StateId stateId(keypointId.frameId);
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(stateId),
                    "state does not exists")
  State& state = states_.at(stateId);
  OKVIS_ASSERT_TRUE_DBG(Exception, state.observations.count(keypointId),
                    "observation does not exists")
  state.observations.erase(keypointId);
  observations_.erase(keypointId);

  // covisibilities invalid
  covisibilitiesComputed_ = false;

  return true;
}

bool ViGraph::computeCovisibilities()
{
  if(covisibilitiesComputed_) {
    return true; // already done previously
  }
  coObservationCounts_.clear();
  visibleFrames_.clear();
  for(auto iter=landmarks_.begin(); iter!=landmarks_.end(); ++iter) {
    if(iter->second.classification == 10 || iter->second.classification == 11) {
      continue;
    }
    auto obs = iter->second.observations;
    std::set<uint64> covisibilities;
    for(auto obsiter=obs.begin(); obsiter!=obs.end(); ++obsiter) {
      covisibilities.insert(obsiter->first.frameId);
      visibleFrames_.insert(StateId(obsiter->first.frameId));
    }
    for(auto i0=covisibilities.begin(); i0!=covisibilities.end(); ++i0) {
      for(auto i1=covisibilities.begin(); i1!=covisibilities.end(); ++i1) {
        if(*i1>=*i0) {
          continue;
        }
        if(coObservationCounts_.find(*i0)==coObservationCounts_.end()) {
          coObservationCounts_[*i0][*i1] = 1;
        } else {
          if (coObservationCounts_.at(*i0).find(*i1)==coObservationCounts_.at(*i0).end()) {
            coObservationCounts_.at(*i0)[*i1] = 1;
          } else {
            coObservationCounts_.at(*i0).at(*i1)++;
          }
        }
      }
    }
  }
  covisibilitiesComputed_ = true;
  return coObservationCounts_.size()>0;
}



int ViGraph::covisibilities(StateId pose_i, StateId pose_j) const
{
  OKVIS_ASSERT_TRUE(Exception, covisibilitiesComputed_, "covisibilities not yet computed")
  if(pose_i==pose_j) {
    return 0;
  }
  size_t a=pose_i.value();
  size_t b=pose_j.value();
  if(pose_i < pose_j) {
    b=pose_i.value();
    a=pose_j.value();
  }
  if(coObservationCounts_.count(a)) {
    if(coObservationCounts_.at(a).count(b)) {
      return coObservationCounts_.at(a).at(b);
    }
  }
  return 0;
}

bool ViGraph::addRelativePoseConstraint(StateId poseId0, StateId poseId1,
                                         const kinematics::Transformation &T_S0S1,
                                         const Eigen::Matrix<double, 6, 6> &information)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(poseId0), "stateId not found")
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(poseId1), "stateId not found")
  State & state0 = states_.at(poseId0);
  State & state1 = states_.at(poseId1);
  OKVIS_ASSERT_TRUE_DBG(Exception, state0.relativePoseLinks.count(poseId1)==0,
                    "relative pose error already exists")
  OKVIS_ASSERT_TRUE_DBG(Exception, state1.relativePoseLinks.count(poseId0)==0,
                    "relative pose error already exists")
  RelativePoseLink relativePoseLink;
  relativePoseLink.errorTerm.reset(new ceres::RelativePoseError(information, T_S0S1));
  relativePoseLink.residualBlockId = problem_->AddResidualBlock(relativePoseLink.errorTerm.get(),
                                                                nullptr, state0.pose->parameters(),
                                                                state1.pose->parameters());
  relativePoseLink.state0 = poseId0;
  relativePoseLink.state1 = poseId1;
  // add to book-keeping
  state0.relativePoseLinks[poseId1] = relativePoseLink;
  state1.relativePoseLinks[poseId0] = relativePoseLink;

  return true;
}

bool ViGraph::removeRelativePoseConstraint(StateId poseId0, StateId poseId1)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(poseId0),
                        "stateId " << poseId0.value() << " not found")
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(poseId1),
                        "stateId " << poseId1.value() << "not found")
  State & state0 = states_.at(poseId0);
  State & state1 = states_.at(poseId1);
  OKVIS_ASSERT_TRUE_DBG(Exception, state0.relativePoseLinks.count(poseId1)==1,
                    "relative pose error does not exists")
  OKVIS_ASSERT_TRUE_DBG(Exception, state1.relativePoseLinks.count(poseId0)==1,
                    "relative pose error does not exists")
  RelativePoseLink & relativePoseLink = state0.relativePoseLinks.at(poseId1);
  problem_->RemoveResidualBlock(relativePoseLink.residualBlockId);

  // add to book-keeping
  state0.relativePoseLinks.erase(poseId1);
  state1.relativePoseLinks.erase(poseId0);

  return true;
}

void ViGraph::freezeGpsExtrinsics(){
    problem_->SetParameterBlockConstant(states_.begin()->second.T_GW->parameters());
    gpsFixed_ = true;
    std::cout << "[DEBUG] FREEZING GPS Extrinsics!" << std::endl;
}

bool ViGraph::setGpsExtrinsics(const kinematics::TransformationCacheless & T_GW){
    states_.begin()->second.T_GW->setEstimate(T_GW);
    return true;
}

bool ViGraph::needsGpsReInit(){
    if(gpsStates_.empty())
        return false;
    if(gpsParametersVec_.back().gpsFusionMode == 1 || gpsParametersVec_.back().gpsFusionMode == 2)
      return false;

    StateId lastGpsStateId;
    lastGpsStateId = *(gpsStates_.rbegin());

    if(gpsStatus_ == gpsStatus::Initialised && states_.at(lastGpsStateId).pose->fixed()){
        needsPositionAlignment_ = true;
        gpsDropoutId_ = lastGpsStateId;
        std::cout << "[DEBUG INFO GPS Alignment] Long time GPS dropout. Need to re-initialise." << std::endl;
        return true;
      }
    if(gpsStatus_ == gpsStatus::ReInitialising  && states_.lower_bound(positionAlignedId_)->second.pose->fixed()){
        std::cout << "[DEBUG INFO GPS Alignment] Long time GPS dropout. Need to re-initialise. Last one only position aligned. Full Alignment will reach back..." << std::endl;
        needsPositionAlignment_ = true;
        return true;
      }

    return false;
}

void ViGraph::reInitGpsExtrinsics(){

    gpsStatus_ = gpsStatus::ReInitialising;
    needsPositionAlignment_ = true;
}

bool ViGraph::needsGpsAlignment(StateId& gpsLossId, std::set<StateId>& reInitStates){
    gpsLossId = gpsDropoutId_;
    reInitStates = gpsReInitStates_;
    return needsFullAlignment_;
}

bool ViGraph::needsFullGpsAlignment(StateId& gpsLossId, StateId& gpsAlignId, okvis::kinematics::Transformation& T_GW_new){
    if(needsFullAlignment_){
       gpsLossId = gpsDropoutId_;
       gpsAlignId = *gpsReInitStates_.rbegin();// gpsInitMap_.rbegin()->first;
       checkForGpsInit(T_GW_new, gpsReInitStates_);
       return true;
      }
    else{
        return false;
      }
}

bool ViGraph::needsPosGpsAlignment(StateId& gpsLossId, StateId& gpsAlignId, Eigen::Vector3d& posError){

    if(needsPositionAlignment_){

        gpsLossId = gpsDropoutId_;

        // this check should not be needed
        //std::cout << gpsInitMap_.size() << std::endl;
        //if(gpsInitMap_.size() == 0) {
        //  return false;
        //}

        // obtain last measurement
        GpsMeasurement lastMeasurement = gpsInitMap_.rbegin()->second;
        auto rIter = states_.rbegin();

        while((lastMeasurement.timeStamp < rIter->second.timestamp) && (rIter != states_.rend()))
          ++rIter;

        if(rIter == states_.rend())
          return false;

        okvis::ceres::GpsErrorAsynchronous test(lastMeasurement.measurement.position, lastMeasurement.measurement.covariances,
                                                gpsInitImuQueue_, imuParametersVec_.back(), rIter->second.timestamp, lastMeasurement.timeStamp,
                                                gpsParametersVec_.back());
        double* parameters[3];
        parameters[0]=states_.at(rIter->first).pose->parameters();
        parameters[1]=states_.at(rIter->first).speedAndBias->parameters();
        parameters[2]=states_.begin()->second.T_GW->parameters();
        Eigen::Matrix<double,3,1> residuals;
        okvis::kinematics::Transformation T_GW_original = states_.begin()->second.T_GW->estimate();

        test.EvaluateWithMinimalJacobians(parameters, residuals.data(),NULL,NULL);
        Eigen::Vector3d posError_G = test.error();
        /*std::cout << "corresponding uncertainties are: \n" << lastMeasurement.measurement.covariances.diagonal().transpose() << std::endl;
        if(posError_G(0) < std::sqrt(lastMeasurement.measurement.covariances(0,0)) ||
          posError_G(1) < std::sqrt(lastMeasurement.measurement.covariances(1,1)) ||
          posError_G(2) < std::sqrt(lastMeasurement.measurement.covariances(2,2))){
          std::cout << "position alignment not applied. uncertainty too large" << std::endl;
          return false;
        }*/
        //Eigen::Vector3d posError_G(residuals[0], residuals[1], residuals[2]);
        posError = T_GW_original.inverse().C() * posError_G;

        gpsAlignId = rIter->first;
        positionAlignedId_ = rIter->first;

        return true;

      }
    else{
        return false;
      }
}

bool ViGraph::needsInitialGpsAlignment(){
  return needsInitialAlignment_;
}


void ViGraph::resetGpsAlignment(){

    // re-set T_GW parameter blocks
    auto iterSid = states_.find(*(gpsReInitStates_.begin()));
    for( ; iterSid != states_.end(); iterSid++){
        StateId sid = iterSid->first;
        states_.at(sid).T_GW = states_.at(StateId(1)).T_GW; // set all to original T_GW
        for(auto iterFac = states_.at(sid).GpsFactors.begin() ; iterFac != states_.at(sid).GpsFactors.end() ; iterFac++){
            ::ceres::ResidualBlockId rid = iterFac->residualBlockId;
            problem_->RemoveResidualBlock(rid);
            iterFac->residualBlockId = problem_->AddResidualBlock(iterFac->errorTerm.get(), nullptr,
                                                             states_.at(sid).pose->parameters(),states_.at(sid).speedAndBias->parameters(), states_.at(sid).T_GW->parameters());
        }
    }
    gpsReInitialised_ = false;
    gpsStatus_ = gpsStatus::Initialised;
    gpsReInitStates_.clear();
    std::cout << "[DEBUG Info Gps Alignment] reset alignment status"  << std::endl;
}

void ViGraph::resetFullGpsAlignment(){

    gpsReInitialised_ = false;
    needsFullAlignment_ = false;
    needsPositionAlignment_ = false; // for the unlikely case that position and full alignment become available at the same time
    gpsStatus_ = gpsStatus::Initialised;
    // gpsInitMap_.clear();
    gpsInitImuQueue_.clear();
}

void ViGraph::resetPosGpsAlignment(){
    needsPositionAlignment_=false;
}

bool ViGraph::addGpsMeasurement(StateId poseId, GpsMeasurement &gpsMeas, const ImuMeasurementDeque &imuMeasurements){

//        std::cout << "[DEBUG INFO ViGraph] Trying to add gps Measurement at t = " << gpsMeas.timeStamp << " to state with t = " << states_.at(poseId).timestamp
//                  << "and imu measurements ranging from " << imuMeasurements.front().timeStamp << " to " << imuMeasurements.back().timeStamp << std::endl;
//        std::cout << " State has ID: " << poseId.value() << std::endl;
    OKVIS_ASSERT_TRUE(Exception, states_.count(poseId), "stateId " << poseId.value() << " not found")
    OKVIS_ASSERT_TRUE(Exception, (gpsMeas.timeStamp >= states_.at(poseId).timestamp), "GPS measurement too old to add to state" )
    if(!(imuMeasurements.front().timeStamp <= states_.at(poseId).timestamp)){
      LOG(WARNING) << "IMU Measurements for adding GPS error are not old enough" << std::endl;
      return false;
    }
    //OKVIS_ASSERT_TRUE(Exception, (imuMeasurements.front().timeStamp <= states_.at(poseId).timestamp), "IMU Measurements for adding GPS error are not old enough");
    OKVIS_ASSERT_TRUE(Exception, (imuMeasurements.back().timeStamp >= gpsMeas.timeStamp), "IMU measurements do not cover GPS measurement");

    gpsStates_.insert(poseId); // Save states that carry gps measurements
    State & state = states_.at(poseId); // Obtain reference to state
    GpsFactor newGpsFactor; // Create new GPS error term

    // If raw geodetic measurements are fed, convert measurements
    if(gpsParametersVec_.back().type == "geodetic" || gpsParametersVec_.back().type == "geodetic-leica"){
      double x,y,z;
      globCartesianFrame_.Forward(gpsMeas.measurement.latitude, gpsMeas.measurement.longitdue, gpsMeas.measurement.height,
                                  x, y, z);
      gpsMeas.measurement.setPosition(x, y, z);
    }

    // Robust loss
    ::ceres::CauchyLoss cauchyLoss(3);

    newGpsFactor.errorTerm.reset(new ceres::GpsErrorAsynchronous(gpsMeas.measurement.position, gpsMeas.measurement.covariances.inverse(),
                                     imuMeasurements, imuParametersVec_.back(),state.timestamp, gpsMeas.timeStamp, gpsParametersVec_.back()));
    newGpsFactor.residualBlockId = problem_->AddResidualBlock(newGpsFactor.errorTerm.get(), nullptr,
                                                                   state.pose->parameters(),state.speedAndBias->parameters(), state.T_GW->parameters());
    state.GpsFactors.push_back(newGpsFactor);

    return true;

}

bool ViGraph::checkForGpsInit(okvis::kinematics::Transformation& T_GW, std::set<StateId> consideredStates) {

  if(consideredStates.size() < 2)
    return false;

  std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > gpsPoints;
  std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> worldPoints;
  std::deque<Eigen::Matrix<double,3,3>, Eigen::aligned_allocator<Eigen::Matrix<double,3,3>> > covariances;

  // Get all measurements so far as well as corresponding propagated poses
  // Iterate over all occuring states with gps measurements
  for(auto gpsState : consideredStates){
    //Iterate measurements per state
    okvis::kinematics::Transformation T_WS_state = states_.at(gpsState).pose->estimate();
    okvis::SpeedAndBias sb_state = states_.at(gpsState).speedAndBias->estimate();

    for(auto iter = states_.at(gpsState).GpsFactors.begin(); iter != states_.at(gpsState).GpsFactors.end(); ++iter){
      gpsPoints.push_back(iter->errorTerm->measurement());
      okvis::kinematics::Transformation T_WS_prop;
      iter->errorTerm->applyPreInt(T_WS_state,sb_state,T_WS_prop);
      worldPoints.push_back(T_WS_prop.r() + T_WS_prop.C() * gpsParametersVec_.back().r_SA);
      covariances.push_back(iter->errorTerm->covariance());
    }
  }

  // align based on SVD: source http://nghiaho.com/?page_id=671

  // compute centroids (A <-> world, B <-> gps)
  Eigen::Vector3d centroidGps, centroidWorld;
  Eigen::MatrixXd gpsPtMatrix(3, gpsPoints.size());
  Eigen::MatrixXd worldPtMatrix(3, worldPoints.size());
  centroidGps.setZero();
  centroidWorld.setZero();
  gpsPtMatrix.setZero();
  worldPtMatrix.setZero();

  for(size_t i = 0; i < gpsPoints.size() ; ++i){
    gpsPtMatrix.col(i) = gpsPoints.at(i);
    worldPtMatrix.col(i) = worldPoints.at(i);

    centroidGps += gpsPoints.at(i);
    centroidWorld += worldPoints.at(i);

  }
  centroidGps /= gpsPoints.size();
  centroidWorld /= worldPoints.size();



  // build H matrix
  gpsPtMatrix.colwise() -= centroidGps;
  worldPtMatrix.colwise() -= centroidWorld;

  Eigen::Matrix3d H;
  H = worldPtMatrix * gpsPtMatrix.transpose();

  double A = H(0,1) - H(1,0);
  double B = H(0,0) + H(1,1);
  double theta = M_PI / 2.0 - std::atan2(B,A);
  Eigen::Matrix3d R_yaw;
  R_yaw.setZero();
  R_yaw(0,0)=std::cos(theta);
  R_yaw(0,1) = - std::sin(theta);
  R_yaw(1,0) = std::sin(theta);
  R_yaw(1,1) = std::cos(theta);
  R_yaw(2,2) = 1.0;

  // translation
  Eigen::Vector3d t = centroidGps - R_yaw * centroidWorld;

  Eigen::Matrix4d init;
  init.setIdentity();
  init.topLeftCorner<3,3>() = R_yaw;
  init.topRightCorner<3,1>() = t;

  //okvis::kinematics::Transformation T_GW;//(init);
  T_GW.set(init);
  //std::cout << "Initialisation procedure leads to \n" << T_GW.T3x4() << std::endl;

  // compute yaw uncertainty
  Eigen::Matrix<double,4,4> Hess;
  Hess.setZero();
  // Get all measurements so far as well as corresponding propagated poses
  Eigen::Matrix<double,3,4> Ei;
  Eigen::Matrix<double,4,4> tmp2;
  for(size_t i = 0; i < worldPoints.size(); ++i){

    Eigen::Vector3d pt = worldPoints.at(i);
    Ei.setZero();
    Ei.topLeftCorner<3,3>() = -Eigen::Matrix3d::Identity();
    Eigen::Matrix<double,3,3> tmp = okvis::kinematics::crossMx(T_GW.C()*pt);
    Ei.topRightCorner<3,1>() = tmp.col(2);
    tmp2 = Ei.transpose() * covariances.at(i).inverse() * Ei;
    Hess = Hess + tmp2;
  }

  // invert hessian
  Eigen::Matrix<double,4,4> P = Hess.inverse();
  double yawUncertainty = std::sqrt(P(3,3)) / M_PI * 180.0;
  std::cout << "[Debug GPS initialisation] estimated yaw uncertainty [degree]: "
            << yawUncertainty << std::endl;
  //std::cout << "[Debug GPS initialisation] Covariance matrix.: \n"
  //          << P << std::endl;


  if(yawUncertainty < gpsParametersVec_.back().yawErrorThreshold)
    return true;
  else
    return false;
}

void ViGraph::initialiseGpsExtrinsics(okvis::kinematics::Transformation& T_GW){

  std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gpsPoints;
  std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> worldPoints;

  std::deque<StateId> sids;

  // Iterate over all occuring states with gps measurements
  for(auto gpsState : gpsStates_){
    //Iterate measurements per state
    okvis::kinematics::Transformation T_WS_state = states_.at(gpsState).pose->estimate();
    okvis::SpeedAndBias sb_state = states_.at(gpsState).speedAndBias->estimate();

    for(auto iter = states_.at(gpsState).GpsFactors.begin(); iter != states_.at(gpsState).GpsFactors.end(); ++iter){
      gpsPoints.push_back(iter->errorTerm->measurement());
      okvis::kinematics::Transformation T_WS_prop;
      iter->errorTerm->applyPreInt(T_WS_state,sb_state,T_WS_prop);
      worldPoints.push_back(T_WS_prop.r() + T_WS_prop.C() * gpsParametersVec_.back().r_SA);
    }
  }

  // align based on SVD: source http://nghiaho.com/?page_id=671

  // compute centroids (A <-> world, B <-> gps)
  Eigen::Vector3d centroidGps, centroidWorld;
  Eigen::MatrixXd gpsPtMatrix(3, gpsPoints.size());
  Eigen::MatrixXd worldPtMatrix(3, worldPoints.size());
  centroidGps.setZero();
  centroidWorld.setZero();
  gpsPtMatrix.setZero();
  worldPtMatrix.setZero();


  for(size_t i = 0; i < gpsPoints.size() ; ++i){
    gpsPtMatrix.col(i) = gpsPoints.at(i);
    worldPtMatrix.col(i) = worldPoints.at(i);

    centroidGps += gpsPoints.at(i);
    centroidWorld += worldPoints.at(i);
  }
  centroidGps /= gpsPoints.size();
  centroidWorld /= worldPoints.size();

  // build H matrix
  gpsPtMatrix.colwise() -= centroidGps;
  worldPtMatrix.colwise() -= centroidWorld;

  Eigen::Matrix3d H;
  H = worldPtMatrix * gpsPtMatrix.transpose();

  double A = H(0,1) - H(1,0);
  double B = H(0,0) + H(1,1);
  double theta = M_PI / 2.0 - std::atan2(B,A);
  Eigen::Matrix3d R_yaw;
  R_yaw.setZero();
  R_yaw(0,0)=std::cos(theta);
  R_yaw(0,1) = - std::sin(theta);
  R_yaw(1,0) = std::sin(theta);
  R_yaw(1,1) = std::cos(theta);
  R_yaw(2,2) = 1.0;

  // translation
  Eigen::Vector3d t = centroidGps - R_yaw * centroidWorld;

  Eigen::Matrix4d init;
  init.setIdentity();
  init.topLeftCorner<3,3>() = R_yaw;
  init.topRightCorner<3,1>() = t;

  T_GW.set(init);
}

bool ViGraph::addGpsMeasurements(GpsMeasurementDeque& gpsMeasurementDeque, ImuMeasurementDeque& imuMeasurementDeque, std::deque<StateId>* sids){

  // Make sure that IMU measurements span larger time interval
  if(gpsMeasurementDeque.size() == 0 || imuMeasurementDeque.size() == 0){
      return false;
  }

  OKVIS_ASSERT_TRUE(Exception, (imuMeasurementDeque.front().timeStamp <= gpsMeasurementDeque.front().timeStamp), "IMU measurements not old enough for gps measurement");
  OKVIS_ASSERT_TRUE(Exception, (imuMeasurementDeque.back().timeStamp >= gpsMeasurementDeque.back().timeStamp), "GPS measurement not covered by IMU measurements");

  StateId sid; // IDs of states that GPS Measurements are added to (needed to log for fullgraph optimisation)
  if(sids != nullptr)
      sids->clear();

  if(gpsStatus_ == gpsStatus::Off || gpsStatus_ == gpsStatus::Idle || gpsStatus_ == gpsStatus::Initialising || gpsStatus_ == gpsStatus::ReInitialising){

      // Save IMU Measurements for observability consideration
      for(auto imuIter = imuMeasurementDeque.begin() ; imuIter != imuMeasurementDeque.end() ; ++imuIter){
          if(!gpsInitImuQueue_.empty() && (imuIter->timeStamp < gpsInitImuQueue_.back().timeStamp))
            continue;
          gpsInitImuQueue_.push_back(*imuIter);
        }
    }

  // Iterator to traverse states backwards
  auto rIterStates = states_.rbegin();
  // Reverse iterate measurements
  for(auto rIterMeas = gpsMeasurementDeque.rbegin(); rIterMeas != gpsMeasurementDeque.rend() ; rIterMeas++){
      // Reverse iterate States to find corresponding state / measurement pairs
      while(rIterStates->second.timestamp > rIterMeas->timeStamp && rIterStates != states_.rend()){
          rIterStates++;
      }

      // If state iterator comes to begin, measurement cannot be added
      if(rIterStates == states_.rend()){
          break;
      }

      // State ID determined where GPS measurement has to be added
      sid = rIterStates->first;
      if(sids != nullptr)
        sids->push_front(sid);

      // Save gps Status for evaluation purposes
      states_.at(sid).gpsMode = gpsStatus_;
      // Initialise variables vor xy delta of measurements and GPS reference frame
      Eigen::Vector2d xyDelta;
      okvis::kinematics::Transformation T_GW_init;

      switch(gpsStatus_){

        case gpsStatus::Off : // no GPS measurements received so far

          // Initialize Cartesian reference frame
          globCartesianFrame_.Reset(rIterMeas->measurement.latitude, rIterMeas->measurement.longitdue,
                                    rIterMeas->measurement.height);

          // Save Measurements for observability consideration
          gpsInitMap_.insert({sid, *rIterMeas});
          std::cout << "[GPS Info] First measurements added. Starting GPS-VIO Initialisation." << std::endl;
          gpsStatus_=gpsStatus::Idle;
          break;

        case gpsStatus::Idle : // waiting for first initialisation of GPS-VIO extrinsics
          gpsInitMap_.insert({sid, *rIterMeas});

          if(gpsInitMap_.size() >= 2){
            std::cout << "[GPS Info] GPS-VIO extrinsics initialised for first time. Starting to add measurements... " << std::endl;
            initialiseGpsExtrinsics(T_GW_init);
            setGpsExtrinsics(T_GW_init);
            for(auto iterQueue = gpsInitMap_.begin() ; iterQueue != gpsInitMap_.end() ; ++iterQueue){
              addGpsMeasurement(iterQueue->first , iterQueue->second , gpsInitImuQueue_);
            }

            if(gpsParametersVec_.back().gpsFusionMode == 1){ // Only Initialise once
              gpsStatus_ = gpsStatus::Initialised;
            }
            else{ // Always doing SVD for initialisation or observability purposes
              gpsStatus_ = gpsStatus::Initialising;
            }
          }

          break;

        case gpsStatus::Initialising : // initialization state

          // Save Measurements for observability consideration
          if(gpsInitMap_.size() > 0){
            xyDelta = gpsInitMap_.rbegin()->second.measurement.position.head<2>() - rIterMeas->measurement.position.head<2>();
            if(std::abs(xyDelta(0)) > std::sqrt(rIterMeas->measurement.covariances(0,0)) ||
               std::abs(xyDelta(1)) > std::sqrt(rIterMeas->measurement.covariances(1,1))){
              gpsInitMap_.insert({sid, *rIterMeas});
            }
          }
          else{
            gpsInitMap_.insert({sid, *rIterMeas});
          }
          addGpsMeasurement(sid, *rIterMeas, imuMeasurementDeque);

          // in fusion mode 1 this gps mode should never be reached
          if(gpsParametersVec_.back().gpsFusionMode == 2){
            // Always provide SVD solution as initial guess
            initialiseGpsExtrinsics(T_GW_init);
            setGpsExtrinsics(T_GW_init);
          }
          else if(gpsParametersVec_.back().gpsFusionMode == 4){
            gpsObservability_ = checkForGpsInit(T_GW_init,gpsStates_);
            setGpsExtrinsics(T_GW_init);
            if(gpsObservability_){
              gpsStatus_ = gpsStatus::Initialised;
              needsInitialAlignment_ = true;
              std::cout << "[GPS Info] GPS-VIO extrinsics have become observable." << std::endl;
            }
          }

          break;

        case gpsStatus::Initialised : // Initialised state, simply add measurements
          addGpsMeasurement(sid, *rIterMeas, imuMeasurementDeque);
          break;

        case gpsStatus::ReInitialising : // re-initialisation stage

          // Save Measurements for observability consideration
          gpsInitMap_.insert({sid, *rIterMeas});
          gpsReInitStates_.insert(sid);
          // Add Measurement
          addGpsMeasurement(sid, *rIterMeas, imuMeasurementDeque);

          //gpsReInitialised_ = computeGpsObservability();
          gpsReInitialised_ = checkForGpsInit(T_GW_init,gpsReInitStates_);
          if(gpsReInitialised_){
              needsFullAlignment_ = true;
              std::cout << "[GPS Info] GPS fully re-Initialised. Ready to apply global alignment." << std::endl;
              gpsStatus_ = gpsStatus::Initialising;
            }

          break;

        default:
            std::cout << "GPS ERROR. Mode wrongly configured" << std::endl;
            return false;
        }

  }

  return true;
}

// Obtain the Hessian block for a specific (landmark) parameter block.
void ViGraph::getLandmarkHessian(LandmarkId lmId, Eigen::Matrix3d& H) {
  OKVIS_ASSERT_TRUE_DBG(Exception,landmarks_.count(lmId) ,"parameter block not in map.");
  // Set Hessian initially to 0
  H.setZero();

  // obtain residual blocks for landmark of interest
  // std::vector<::ceres::ResidualBlockId> residualBlocks;
  ::ceres::ResidualBlockId resBlock;
  KeypointIdentifier kid;

  for(auto iter = landmarks_.at(lmId).observations.begin(); iter != landmarks_.at(lmId).observations.end(); ++iter){
      // residual block
      kid = iter->first;
      resBlock = iter->second.residualBlockId;

      // parameters & jacobians
      Eigen::Matrix<double,2,1> residuals;
      //double* residuals;
      std::vector<double*> parametersVec;
      double* parameters[3];
      double* jacobians[3];
      Eigen::Matrix<double,2,7,Eigen::RowMajor> J0;
      Eigen::Matrix<double,2,4,Eigen::RowMajor> J1;
      Eigen::Matrix<double,2,7,Eigen::RowMajor> J2;
      jacobians[0]=J0.data();
      jacobians[1]=J1.data();
      jacobians[2]=J2.data();
      double* jacobiansMinimal[3];
      Eigen::Matrix<double,2,6,Eigen::RowMajor> J0min;
      Eigen::Matrix<double,2,3,Eigen::RowMajor> J1min;
      Eigen::Matrix<double,2,6,Eigen::RowMajor> J2min;
      jacobiansMinimal[0]=J0min.data();
      jacobiansMinimal[1]=J1min.data();
      jacobiansMinimal[2]=J2min.data();
      problem_->GetParameterBlocksForResidualBlock(resBlock,&parametersVec); // fill parameters
      parameters[0] = parametersVec[0];
      parameters[1] = parametersVec[1];
      parameters[2] = parametersVec[2];

      // Evaluate Residualblock
      landmarks_.at(lmId).observations.at(kid).errorTerm->EvaluateWithMinimalJacobians(
                  parameters,residuals.data(), jacobians, jacobiansMinimal);
      Eigen::Map< Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jl(jacobiansMinimal[1]);


      // Update Hessian
      H+= Jl.transpose() * Jl;

      // cleanup
//          delete[] parameters;
//          delete[] jacobians;
//          delete[] jacobiansMinimal;
  }
}

bool ViGraph::computeGpsObservability(/*double threshold --> moved into parameter struct */ ){

    if(gpsInitMap_.empty())
      return false;

    // Compute T_GW observability depending on so-far occuring measurements
    Eigen::Vector2d mean(0.0,0.0); // Initialisation of weighted mean
    Eigen::Matrix2d Wnormal; // for weighted mean: "normalization constant"
    Wnormal.setZero();

    for(auto iter = gpsInitMap_.begin() ; iter != gpsInitMap_.end() ; ++iter){
        mean += (iter->second.measurement.covariances.topLeftCorner<2,2>().inverse() * iter->second.measurement.position.head<2>());
        Wnormal += (iter->second.measurement.covariances.topLeftCorner<2,2>().inverse());
      }

    // final weighte sample mean
    mean = Wnormal.inverse()*mean;

    // Iter measurements again for weighted sample covariance computation
    double varSum = 0;
    for (auto iter = gpsInitMap_.begin() ; iter != gpsInitMap_.end() ; ++iter){
        // e^T * W * e
        varSum += ( (iter->second.measurement.position.head<2>() - mean).transpose() * iter->second.measurement.covariances.topLeftCorner<2,2>().inverse() * (iter->second.measurement.position.head<2>() - mean) );
    }
    varSum /= gpsInitMap_.size();

    // new approach using chi-square distribution
    double c = 0;
    //std::cout << "[DEBUG GPS Initialisation] Computing chi square statistics..." << std::endl;
    //std::cout << "[------------------------] Number of measurements: " << gpsInitMap_.size() << std::endl;
    //int ndof = gpsInitMap_.size() - 4;
    //std::cout << "[------------------------] ndof: " << ndof << std::endl;
    //std::cout << "[------------------------] weighted mean: " << mean.transpose() << std::endl;
    //double chi_square_sum = 0.0;

    for (auto iter = gpsInitMap_.begin() ; iter != gpsInitMap_.end() ; ++iter){
        // e^T * W * e
        double chi_square_error = ( (iter->second.measurement.position.head<2>() - mean).transpose() * iter->second.measurement.covariances.topLeftCorner<2,2>().inverse() * (iter->second.measurement.position.head<2>() - mean) );
        //chi_square_sum += chi_square_error;
        //std::cout << "[------------------------] measurement: " << iter->second.measurement.position.head<2>().transpose() << std::endl;
        //std::cout << "[------------------------] chi-square error: " << chi_square_error << std::endl;
        c += std::max(0.0, chi_square_error - gpsParametersVec_.back().gpsMeasVarianceThreshold);
    }
    //std::cout << "[------------------------] fit quality: " << chi_square_sum / static_cast<double>(gpsInitMap_.size()) << std::endl;
    //boost::math::chi_squared dist(gpsInitMap_.size());
    //std::cout << "[------------------------] cdf value: " << 1. - cdf(dist,chi_square_sum) << std::endl;
    //std::cout << "[------------------------] cdf value: " << 1. - cdf(dist,chi_square_sum) << std::endl;
    //std::cout << "c = " << c << " with " << gpsInitMap_.size() << " measurements" << std::endl;

   if( c > gpsParametersVec_.back().gpsObservabilityThreshold)
        return true;
    else
      return false;
    //if(ndof > 0 && (chi_square_sum / static_cast<double>(gpsInitMap_.size())) > 4.0)
    //  return true;
    //else
    //  return false;

//    // Iter gps measurements for computation of mean
//    for(auto gpsMeas : gpsInitQueue_){
//        // non weighted: mean += gpsMeas.measurement.position.head<2>();
//        // weighted
//        mean += (gpsMeas.measurement.covariances.topLeftCorner<2,2>().inverse() * gpsMeas.measurement.position.head<2>());
//        Wnormal += gpsMeas.measurement.covariances.topLeftCorner<2,2>().inverse();
//    }
//    // final weighte sample mean
//    mean = Wnormal.inverse()*mean;

//    // Iter measurements again for weighted sample covariance computation
//    double varSum = 0;
//    for (auto gpsMeas : gpsInitQueue_){
//        // e^T * W * e
//        varSum += ( (gpsMeas.measurement.position.head<2>() - mean).transpose() * gpsMeas.measurement.covariances.topLeftCorner<2,2>().inverse() * (gpsMeas.measurement.position.head<2>() - mean) );
//    }
//    varSum /= gpsInitQueue_.size();

//    double stdDev = std::sqrt(varSum);

//    //std::cout << "[DEBUG INFO GPS Measurements] xy weighted stdev for " << residualIds.size() << " occuring measurements: " << stdDev << " with " << states_.size() << " states." << std::endl;

//    if(stdDev > gpsParametersVec_.back().gpsObservabilityThreshold)
//        return true;
//    else
//      return false;
}
bool ViGraph::computeGpsObservability(okvis::ceres::PoseParameterBlock& gpsExtrinsics ){

    std::vector<::ceres::ResidualBlockId> residualIds;
    problem_->GetResidualBlocksForParameterBlock(gpsExtrinsics.parameters(), &residualIds);

    okvis::GpsMeasurementDeque measurementDeque;

    for(auto res : residualIds){
        const ::ceres::CostFunction* costFct;
        Eigen::Vector3d measurement;
        Eigen::Matrix3d measCov;
        okvis::Time tg, tk;

        costFct = problem_->GetCostFunctionForResidualBlock(res);
        measurement = static_cast<okvis::ceres::GpsErrorAsynchronous*>(const_cast<::ceres::CostFunction*>(costFct))->measurement();
        measCov = static_cast<okvis::ceres::GpsErrorAsynchronous*>(const_cast<::ceres::CostFunction*>(costFct))->covariance();
        tg = static_cast<okvis::ceres::GpsErrorAsynchronous*>(const_cast<::ceres::CostFunction*>(costFct))->tg();
        tk = static_cast<okvis::ceres::GpsErrorAsynchronous*>(const_cast<::ceres::CostFunction*>(costFct))->tk();

        okvis::GpsSensorReadings sensRead(measurement, measCov);
        okvis::GpsMeasurement measEl(tg,sensRead,-1);
        measurementDeque.push_back(measEl);
    }

    // compute observability of occuring measurements
    // For initialization / computation of mean / covariance
    Eigen::Vector2d mean(0.0,0.0);
    Eigen::Matrix2d Wnormal; // for weighted mean: "normalization constant"
    Wnormal.setZero();

    // Iter gps measurements for computation of mean
    for(auto gpsMeas : measurementDeque){
        // non weighted: mean += gpsMeas.measurement.position.head<2>();
        // weighted
        mean += (gpsMeas.measurement.covariances.topLeftCorner<2,2>().inverse() * gpsMeas.measurement.position.head<2>());
        Wnormal += gpsMeas.measurement.covariances.topLeftCorner<2,2>().inverse();
    }
   // non-weighted: mean /= measurementDeque.size();
    //weighted
    mean = Wnormal.inverse()*mean;

    // Iter measurements again vor variance computation
    double varSum = 0;
    for (auto gpsMeas : measurementDeque){
        // e^T * W * e
        varSum += ( (gpsMeas.measurement.position.head<2>() - mean).transpose() * gpsMeas.measurement.covariances.topLeftCorner<2,2>().inverse() * (gpsMeas.measurement.position.head<2>() - mean) );
    }
    varSum /= measurementDeque.size();

    double stdDev = std::sqrt(varSum);

    //std::cout << "[DEBUG INFO GPS Measurements] xy weighted stdev for " << residualIds.size() << " occuring measurements: " << stdDev << std::endl;

    if(stdDev > /*threshold*/ gpsParametersVec_.back().gpsObservabilityThreshold)
        return true;
    else
        return false;
}

bool ViGraph::addSubmapAlignmentConstraints(const se::OccupancyMap<se::Res::Multi>* submap_ptr,
                                            const uint64_t &frame_A_id, const uint64_t frame_B_id,
                                            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& pointCloud) {
  std::cout << "[DEBUG] Registered Submap Alignment Callback! \n"
            << " --- frame A: " << frame_A_id << "\n"
            << " --- frame B: " << frame_B_id << "\n"
            << " --- # of points: " << pointCloud.size() << std::endl;

  State & state_A = states_.at(StateId(frame_A_id));
  State & state_B = states_.at(StateId(frame_B_id));
  for(uint64_t i = 0; i < pointCloud.size(); i++){
    //std::cout << i << " / " << pointCloud.size();
    Eigen::Vector3f pt = pointCloud.at(i);
    SubmapAlignmentFactor alignFac;
    ::ceres::CostFunction *cost_function = new okvis::ceres::SubmapIcpError(*submap_ptr, pt.cast<double>(), 0.01);
    problem_->AddResidualBlock(cost_function, nullptr, state_A.pose->parameters(), state_B.pose->parameters());
    //alignFac.errorTerm.reset(new ceres::SubmapIcpError(*submap_ptr, pt.cast<double>(), 0.01));
    //alignFac.residualBlockId = problem_->AddResidualBlock(alignFac.errorTerm.get(), nullptr,
    //                                                      state_A.pose->parameters(), state_B.pose->parameters());
  }
  std::cout << "added all constraints!" << std::endl;
  //ToDo: state_A.SubmapAlignmentFactors.push_back();
  //ToDo: state_B.SubmapAlignmentFactors.push_back();
  return true;
}

const Eigen::Vector4d &ViGraph::landmark(LandmarkId id) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(id), "landmark does not exists")
  return landmarks_.at(id).hPoint->estimate();
}

bool ViGraph::getLandmark(LandmarkId landmarkId, MapPoint2& mapPoint) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId), "landmark does not exists")
  std::set<KeypointIdentifier> observations;
  const Landmark & landmark = landmarks_.at(landmarkId);
  for(auto observation : landmark.observations) {
    observations.insert(observation.first);
  }
  mapPoint = MapPoint2{landmarkId, landmark.hPoint->estimate(), observations,
      landmark.hPoint->initialized(), landmark.quality, landmark.classification};
  return true;
}

size_t ViGraph::getLandmarks(MapPoints &landmarks) const
{
  for(auto& landmark : landmarks_) {
    std::set<KeypointIdentifier> observations;
    for(auto observation : landmark.second.observations) {
      observations.insert(observation.first);
    }
    landmarks[landmark.first] = MapPoint2{
        landmark.first, landmark.second.hPoint->estimate(), observations,
        landmark.second.hPoint->initialized(), landmark.second.quality,
        landmark.second.classification};
  }
  return landmarks.size();
}

bool ViGraph::landmarkExists(LandmarkId landmarkId) const
{
  return landmarks_.count(landmarkId) != 0;
}

bool ViGraph::setLandmark(LandmarkId id, const Eigen::Vector4d &homogeneousPoint, bool initialised)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(id), "landmark does not exists")
  auto& landmark = landmarks_.at(id).hPoint;
  landmark->setEstimate(homogeneousPoint);
  landmark->setInitialized(initialised);
  return true;
}
bool ViGraph::setLandmark(LandmarkId id, const Eigen::Vector4d &homogeneousPoint)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(id), "landmark does not exists")
  auto& landmark = landmarks_.at(id).hPoint;
  landmark->setEstimate(homogeneousPoint);
  return true;
}

const kinematics::TransformationCacheless & ViGraph::pose(StateId id) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  return states_.at(id).pose->estimate();
}

const SpeedAndBias &ViGraph::speedAndBias(StateId id) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  return states_.at(id).speedAndBias->estimate();
}

const kinematics::TransformationCacheless & ViGraph::T_GW() const
{
  return states_.begin()->second.T_GW->estimate();
}

const kinematics::TransformationCacheless & ViGraph::T_GW(StateId id) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  return states_.at(id).T_GW->estimate();
}

const kinematics::TransformationCacheless &ViGraph::extrinsics(StateId id, uchar camIdx) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.at(id).extrinsics.at(camIdx), "extrinsics do not exists")
      return states_.at(id).extrinsics.at(camIdx)->estimate();
}

bool ViGraph::isKeyframe(StateId id) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  return states_.at(id).isKeyframe;
}


Time ViGraph::timestamp(StateId id) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  return states_.at(id).timestamp;
}

StateId ViGraph::currentStateId() const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, !states_.empty(), "no states exist")
  return states_.rbegin()->first;
}

StateId ViGraph::stateIdByAge(size_t age) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.size()>age, "too few states exist")
  auto iter = states_.rbegin();
  for(size_t k=0; k<age; k++) {
    iter++;
  }
  return iter->first;
}

bool ViGraph::setKeyframe(StateId id, bool isKeyframe)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  states_.at(id).isKeyframe = isKeyframe;
  return true;
}

bool ViGraph::setPose(StateId id, const kinematics::TransformationCacheless &pose)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  states_.at(id).pose->setEstimate(pose);
  return true;
}

bool ViGraph::setSpeedAndBias(StateId id, const SpeedAndBias &speedAndBias)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  states_.at(id).speedAndBias->setEstimate(speedAndBias);
  return true;
}

bool ViGraph::setExtrinsics(StateId id, uchar camIdx,
                            const kinematics::TransformationCacheless &extrinsics) const
{
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.count(id), "state does not exists")
  OKVIS_ASSERT_TRUE_DBG(Exception, states_.at(id).extrinsics.at(camIdx), "extrinsics do not exists")
  states_.at(id).extrinsics.at(camIdx)->setEstimate(extrinsics);
  return true;
}

bool ViGraph::setExtrinsicsVariable()
{
  for(size_t i=0; i<cameraParametersVec_.size(); ++i) {
    problem_->SetParameterBlockVariable(states_.begin()->second.extrinsics.at(i)->parameters());
  }
  return true;
}

bool ViGraph::softConstrainExtrinsics(double posStd, double rotStd)
{
  if(states_.begin()->second.extrinsicsPriors.size() == cameraParametersVec_.size())
  for(size_t i=0; i<cameraParametersVec_.size(); ++i) {
    problem_->RemoveResidualBlock(states_.begin()->second.extrinsicsPriors.at(i).residualBlockId);
  }
  states_.begin()->second.extrinsicsPriors.resize(cameraParametersVec_.size());
  for(size_t i=0; i<cameraParametersVec_.size(); ++i) {
    // add a pose prior
    PosePrior& extrinsicsPrior = states_.begin()->second.extrinsicsPriors.at(i);
    extrinsicsPrior.errorTerm.reset(
          new ceres::PoseError(
            states_.begin()->second.extrinsics.at(i)->estimate(), posStd*posStd, rotStd*rotStd));
    extrinsicsPrior.residualBlockId = problem_->AddResidualBlock(
              extrinsicsPrior.errorTerm.get(), nullptr,
          states_.begin()->second.extrinsics.at(i)->parameters());

  }
  return true;
}

void ViGraph::updateLandmarks()
{
  for (auto it = landmarks_.begin(); it != landmarks_.end(); ++it) {
    Eigen::Vector4d hp = it->second.hPoint->estimate();
    const size_t num = it->second.observations.size();
    bool isInitialised = false;
    double quality = 0.0;
    if(num>1) {
      Eigen::Array<double, 3, Eigen::Dynamic> dirs(3,num);
      int o=0;
      //bool bad = false;
      for(const auto& observation : it->second.observations) {
        kinematics::Transformation T_WS, T_SCi;
        const StateId stateId(observation.first.frameId);
        const State & state = states_.at(stateId);
        T_WS = state.pose->estimate();
        T_SCi = state.extrinsics.at(observation.first.cameraIndex)->estimate();
        kinematics::Transformation T_WCi = T_WS*T_SCi;
        Eigen::Vector3d dir = (T_WCi.C()*(T_WCi.inverse()*hp).head<3>());

        // consider only small reprojection errors
        Eigen::Vector2d err;
        double* params[3];
        params[0] = state.pose->parameters();
        params[1] = it->second.hPoint->parameters();
        params[2] = state.extrinsics.at(observation.first.cameraIndex)->parameters();
        observation.second.errorTerm->Evaluate(params, err.data(), nullptr);
        if(err.norm()>2.5) {
          //bad = true;
          continue;
        }
        if(dir.norm()<0.1) {
          //bad = true;
          continue;
        }
        dirs.col(o) = dir.normalized();
        ++o;
      }
      Eigen::Array<double, 3, Eigen::Dynamic> dirso(3,o);
      dirso = dirs.topLeftCorner(3,o);
      Eigen::Vector3d std_dev =
          ((dirso.colwise() - dirso.rowwise().mean()).square().rowwise().sum()).sqrt();
      quality = std_dev.norm();
      if(quality > 0.04) {
        isInitialised = true;
      }
    }
    // update initialisation
    it->second.hPoint->setInitialized(isInitialised);
    it->second.quality = quality;
  }
}

#ifdef USE_OPENMP
void ViGraph::optimise(int maxIterations, int numThreads, bool verbose)
#else
// avoid warning since numThreads unused
void ViGraph::optimise(int maxIterations, int /*numThreads*/, bool verbose)
#warning openmp not detected, your system may be slower than expected
#endif
{
  // assemble options

#ifdef USE_OPENMP
  options_.num_threads = int(numThreads);
#endif
  options_.max_num_iterations = int(maxIterations);

  if (verbose) {
    options_.minimizer_progress_to_stdout = true;
  } else {
    options_.minimizer_progress_to_stdout = false;
  }

  // Compute covariances
//  ::ceres::Covariance::Options covOptions;
//  covOptions.algorithm_type=::ceres::DENSE_SVD;
//  covOptions.null_space_rank=-1;
//  ::ceres::Covariance covariance(covOptions);
//  std::vector<std::pair<const double*, const double*> > covariance_blocks;
//  covariance_blocks.push_back(std::make_pair(states_.at(okvis::StateId(1)).T_GW->parameters(),states_.at(okvis::StateId(1)).T_GW->parameters()));
//  if(covariance.Compute(covariance_blocks, problem_.get())){
//    std::cout << "Covariances successfully computed." << std::endl;
//    double covariance_gg[6*6];
//    covariance.GetCovarianceBlockInTangentSpace(states_.at(okvis::StateId(1)).T_GW->parameters() , states_.at(okvis::StateId(1)).T_GW->parameters() ,covariance_gg);
//    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covOutput(covariance_gg);
//    std::cout << "Norm of covariance blocks: \n"
//              << covOutput.topLeftCorner<3,3>().norm() << "\n"
//              << covOutput.bottomRightCorner<3,3>().norm() << std::endl;
//  }
//  else
//      std::cout << "Covariance computation failed" << std::endl;

  // call solver
  ::ceres::Solve(options_, problem_.get(), &summary_);

  // summary output
  if (verbose) {
    LOG(INFO) << summary_.FullReport();
  }
}

bool ViGraph::setOptimisationTimeLimit(double timeLimit, int minIterations)
{
  if (ceresCallback_ != nullptr) {
    if (timeLimit < 0.0) {
      // no time limit => set minimum iterations to maximum iterations
      ceresCallback_->setMinimumIterations(options_.max_num_iterations);
      return true;
    }
    ceresCallback_->setTimeLimit(timeLimit);
    ceresCallback_->setMinimumIterations(minIterations);
    return true;
  } else if (timeLimit >= 0.0) {
    ceresCallback_ = std::unique_ptr<okvis::ceres::CeresIterationCallback>(
          new okvis::ceres::CeresIterationCallback(timeLimit, minIterations));
    options_.callbacks.push_back(ceresCallback_.get());
    return true;
  }
  // no callback yet registered with ceres.
  // but given time limit is lower than 0, so no callback needed
  return true;
}

int ViGraph::cleanUnobservedLandmarks(std::map<LandmarkId, std::set<KeypointIdentifier> > *removed)
{
  int ctr = 0;
  for (auto it = landmarks_.begin(); it != landmarks_.end(); ) {
    if(it->second.observations.size()<=1) {
      if(removed) {
        (*removed)[it->first] = std::set<KeypointIdentifier>();
      }
      if(it->second.observations.size()==1) {
        if(removed) {
          removed->at(it->first).insert(it->second.observations.begin()->first);
        }
        removeObservation(it->second.observations.begin()->first);       
      }
      problem_->RemoveParameterBlock(it->second.hPoint->parameters());
      it = landmarks_.erase(it);
      ctr++;
    }
    else {
      ++it;
    }
  }
  return ctr;
}

}  // namespace okvis
