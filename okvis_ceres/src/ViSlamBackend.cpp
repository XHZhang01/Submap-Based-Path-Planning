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
 * @file ViSlamBackend.cpp
 * @brief Source file for the Estimator class. This does all the backend work.
 * @author Stefan Leutenegger
 */

#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <okvis/ViSlamBackend.hpp>
#include <okvis/timing/Timer.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

static const double minDeltaT = 2.0; // [sec]
static int numRealtimePoseGraphFrames = 12;
static const int numPoseGraphFrames = 12;

int ViSlamBackend::addCamera(const CameraParameters &cameraParameters)
{
  observationLessGraph_.addCamera(cameraParameters);
  fullGraph_.addCamera(cameraParameters);
  return realtimeGraph_.addCamera(cameraParameters);
}

int ViSlamBackend::addImu(const ImuParameters &imuParameters)
{
  observationLessGraph_.addImu(imuParameters);
  fullGraph_.addImu(imuParameters);
  return realtimeGraph_.addImu(imuParameters);
}

int ViSlamBackend::addGps(const GpsParameters &gpsParameters)
{
  observationLessGraph_.addGps(gpsParameters);
  fullGraph_.addGps(gpsParameters);
  return realtimeGraph_.addGps(gpsParameters);
}

bool ViSlamBackend::addGpsMeasurementsOnAllGraphs(GpsMeasurementDeque& gpsMeasurementDeque, ImuMeasurementDeque& imuMeasurementDeque){

    // (1) Check if GPS Measurements have to be added
    if(gpsMeasurementDeque.size() > 0){

        bool needsGpsReInit;

        // (2) Check if fullGraph_ is accessible and add measurements to graphs
        if(!isLoopClosing_ && !isLoopClosureAvailable_){ // accessible => add measurements to all graphs

            needsGpsReInit = realtimeGraph_.needsGpsReInit(); // check if GPS extrinsics need to be re-initialised after long GPS dropout
            if(needsGpsReInit){
                // re-initialise GPS Extrinsics
                realtimeGraph_.reInitGpsExtrinsics();
                observationLessGraph_.reInitGpsExtrinsics();
                fullGraph_.reInitGpsExtrinsics();

                //now add measurements
                realtimeGraph_.addGpsMeasurements(gpsMeasurementDeque,imuMeasurementDeque,nullptr);
                observationLessGraph_.addGpsMeasurements(gpsMeasurementDeque,imuMeasurementDeque,nullptr);
                fullGraph_.addGpsMeasurements(gpsMeasurementDeque, imuMeasurementDeque,nullptr);

            }
            else{ // no re-initialisation needed
                realtimeGraph_.addGpsMeasurements(gpsMeasurementDeque,imuMeasurementDeque,nullptr);
                observationLessGraph_.addGpsMeasurements(gpsMeasurementDeque,imuMeasurementDeque,nullptr);
                fullGraph_.addGpsMeasurements(gpsMeasurementDeque, imuMeasurementDeque,nullptr);
            }

            // Try alignment if available / required
            tryGpsAlignment();

        }
        else{ // not accessible => save state ids for measurements and buffer gps measurements

            needsGpsReInit = realtimeGraph_.needsGpsReInit(); // check if GPS extrinsics need to be re-initialised after long GPS dropout

            if(needsGpsReInit){
                // re-initialise GPS Extrinsics
                realtimeGraph_.reInitGpsExtrinsics();
                observationLessGraph_.reInitGpsExtrinsics();
              }


            std::deque<StateId> sids; // obtain state ids to add measurements to while buffering them for fullGraph_

            realtimeGraph_.addGpsMeasurements(gpsMeasurementDeque,imuMeasurementDeque,&sids);
            observationLessGraph_.addGpsMeasurements(gpsMeasurementDeque,imuMeasurementDeque,nullptr);
            std::cout << "[DEBUG INFO ViSlamBackend] FullGraph_ optimisation running. Buffering Measurements." << std::endl;
            // GPS MEASUREMENTS BUFFERING
            for(size_t i = 0; i < gpsMeasurementDeque.size(); i++){
                addGpsBacklog_.push_back(AddGpsBacklog{sids.at(i), gpsMeasurementDeque.at(i), imuMeasurementDeque, needsGpsReInit});
            }
            std::cout << "[DEBUG INFO ViSlamBackend] Queue of size " << addGpsBacklog_.size() << std::endl;
        }

        // (3) compute observability based on internal measurements
        if(!gpsObservability_){
            gpsObservability_ = realtimeGraph_.isGpsObservable();
            if(gpsObservability_)
                std::cout << "[DEBUG INFO ViSlamBackend] T_GW HAS BECOME OBSERVABLE" << std::endl;
        }
        return true; // measurements added
    }
    else
        return false; // no measurements could have been added to the graph
}

bool ViSlamBackend::addStates(MultiFramePtr multiFrame, const ImuMeasurementDeque &imuMeasurements,
                              bool asKeyframe)
{
  AuxiliaryState auxiliaryState;
  auxiliaryState.isImuFrame = true;
  auxiliaryState.isKeyframe = asKeyframe;
  if(multiFrames_.empty()) {
    // initialise
    OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "not allowed")
    const StateId id = realtimeGraph_.addStatesInitialise(multiFrame->timestamp(), imuMeasurements,
                                                          multiFrame->cameraSystem());
    multiFrame->setId(id.value());
    observationLessGraph_.addStatesInitialise(multiFrame->timestamp(), imuMeasurements,
                                              multiFrame->cameraSystem());
    fullGraph_.addStatesInitialise(multiFrame->timestamp(), imuMeasurements,
                                              multiFrame->cameraSystem());
    multiFrames_[id] = multiFrame;
    auxiliaryState.loopId = id;
    auxiliaryStates_[id] = auxiliaryState; // for internal book-keeping
    imuFrames_.insert(id);
    return (id.value()==1 && id.isInitialised());
  } else {
    const StateId id = realtimeGraph_.addStatesPropagate(multiFrame->timestamp(), imuMeasurements,
                                                         asKeyframe);
    multiFrame->setId(id.value());
    observationLessGraph_.addStatesPropagate(multiFrame->timestamp(), imuMeasurements, asKeyframe);

    if(isLoopClosing_ || isLoopClosureAvailable_) {
      addStatesBacklog_.push_back(AddStatesBacklog{multiFrame->timestamp(), id, imuMeasurements});
      touchedStates_.insert(id);
    } else {
      // safe to add to the full graph
      fullGraph_.addStatesPropagate(multiFrame->timestamp(), imuMeasurements, asKeyframe);
    }

    multiFrames_[id] = multiFrame;
    auxiliaryState.loopId = id;
    auxiliaryStates_[id] = auxiliaryState; // for internal book-keeping
    imuFrames_.insert(id);
    return id.isInitialised();
  }
}

void ViSlamBackend::printStates(StateId stateId, std::ostream &buffer) const
{
  const ViGraph::State & state = realtimeGraph_.states_.at(stateId);
  if(state.isKeyframe) {
    buffer << "KF ";
  }
  buffer << "pose: ";
  if (state.pose->fixed()) buffer << "(";
  buffer << "id=" << stateId.value() << ":";
  if (state.pose->fixed()) buffer << ")";
  buffer << ", ";
  buffer << "speedAndBias: ";
  if (state.speedAndBias->fixed()) buffer << "(";
  buffer << "id=" << state.speedAndBias->id() << ":";
  if (state.speedAndBias->fixed()) buffer << ")";
  buffer << ", ";
  buffer << "extrinsics: ";
  for (size_t i = 0; i < state.extrinsics.size(); ++i) {
    uint64_t id = state.extrinsics.at(i)->id();
    if (state.extrinsics.at(i)->fixed()) buffer << "(";
    buffer << "id=" << id << ":";
    if (state.extrinsics.at(i)->fixed()) buffer << ")";
    if (i+1 < state.extrinsics.size()) buffer << ", ";
  }
  buffer << std::endl;
}

bool ViSlamBackend::getLandmark(LandmarkId landmarkId, okvis::MapPoint2& mapPoint) const
{
  return realtimeGraph_.getLandmark(landmarkId, mapPoint);
}


size_t ViSlamBackend::getLandmarks(MapPoints & landmarks) const
{
  return realtimeGraph_.getLandmarks(landmarks);
}

double ViSlamBackend::trackingQuality(StateId id) const
{
  const MultiFramePtr frame = multiFrame(id);
  const size_t numFrames = frame->numFrames();
  std::set<LandmarkId> landmarks;
  std::vector<cv::Mat> matchesImg(numFrames);

  // remember matched points
  int intersectionCount = 0;
  int unionCount = 0;
  int matchedPoints = 0;
  for (size_t im = 0; im < numFrames; ++im) {
    const int rows = frame->image(im).rows/10;
    const int cols = frame->image(im).cols/10;
    const double radius = double(std::min(rows,cols))*kptradius_;
    matchesImg.at(im) = cv::Mat::zeros(rows, cols, CV_8UC1);
    const size_t num = frame->numKeypoints(im);
    cv::KeyPoint keypoint;
    for (size_t k = 0; k < num; ++k) {
      frame->getCvKeypoint(im, k, keypoint);
      uint64_t lmId = frame->landmarkId(im, k);
      if (lmId != 0 && realtimeGraph_.landmarkExists(LandmarkId(lmId))) {
        // make sure these are observed elsewhere
        for(const auto & obs : realtimeGraph_.landmarks_.at(LandmarkId(lmId)).observations) {
          if(obs.first.frameId != id.value()) {
            matchedPoints++;
            cv::circle(matchesImg.at(im), keypoint.pt*0.1, int(radius), cv::Scalar(255),
                       cv::FILLED);
            break;
          }
        }
      }
    }
    // one point per image does not count.
    const int pointArea = int(radius*radius*M_PI);
    intersectionCount += std::max(0,cv::countNonZero(matchesImg.at(im)) - pointArea);
    unionCount += rows*cols - pointArea;
  }
  return matchedPoints < 8 ? 0.0 : double(intersectionCount)/double(unionCount);
}

bool ViSlamBackend::setKeyframe(StateId id, bool isKeyframe) {
  auxiliaryStates_.at(id).isKeyframe = isKeyframe;
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedStates_.insert(id);
  } else {
    fullGraph_.setKeyframe(id, isKeyframe);
  }
  observationLessGraph_.setKeyframe(id, isKeyframe);
  return realtimeGraph_.setKeyframe(id, isKeyframe);
}

bool ViSlamBackend::addLandmark(LandmarkId landmarkId, const Eigen::Vector4d &landmark,
                                bool isInitialised)
{
  bool success = realtimeGraph_.addLandmark(landmarkId, landmark, isInitialised);
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedLandmarks_.insert(landmarkId);
  } else {
    success &= fullGraph_.addLandmark(landmarkId, landmark, isInitialised);
  }
  return success;
}

LandmarkId ViSlamBackend::addLandmark(const Eigen::Vector4d &homogeneousPoint, bool initialised)
{
  const LandmarkId lmId = realtimeGraph_.addLandmark(homogeneousPoint, initialised);
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedLandmarks_.insert(lmId);
  } else {
    fullGraph_.addLandmark(lmId, homogeneousPoint, initialised);
  }
  return lmId;
}

bool ViSlamBackend::setLandmark(LandmarkId landmarkId, const Eigen::Vector4d & landmark,
                                bool isInitialised) {
  bool success = realtimeGraph_.setLandmark(landmarkId, landmark, isInitialised);
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedLandmarks_.insert(landmarkId);
  } else {
    fullGraph_.setLandmark(landmarkId, landmark, isInitialised);
  }
  return success;
}
bool ViSlamBackend::setLandmarkClassification(LandmarkId landmarkId, int classification) {
  if(realtimeGraph_.landmarks_.count(landmarkId)==0) {
    return false;
  }
  realtimeGraph_.landmarks_.at(landmarkId).classification = classification;
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedLandmarks_.insert(landmarkId);
  } else {
    fullGraph_.landmarks_.at(landmarkId).classification = classification;
  }
  return true;
}

bool ViSlamBackend::setObservationInformation(
    StateId stateId, size_t camIdx, size_t keypointIdx, const Eigen::Matrix2d & information) {
  KeypointIdentifier kid(stateId.value(), camIdx, keypointIdx);
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedLandmarks_.insert(realtimeGraph_.observations_.at(kid).landmarkId);
    touchedStates_.insert(stateId);
  } else {
    fullGraph_.observations_.at(kid).errorTerm->setInformation(information);
  }
  realtimeGraph_.observations_.at(kid).errorTerm->setInformation(information);
  return true;
}

bool ViSlamBackend::removeObservation(StateId stateId, size_t camIdx,
                                      size_t keypointIdx)
{
  KeypointIdentifier kid(stateId.value(), camIdx, keypointIdx);
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedLandmarks_.insert(realtimeGraph_.observations_.at(kid).landmarkId);
    touchedStates_.insert(stateId);
  } else {
    fullGraph_.removeObservation(kid);
  }
  bool success = realtimeGraph_.removeObservation(kid);
  multiFrames_.at(stateId)->setLandmarkId(camIdx, keypointIdx, 0);
  return success;
}

bool ViSlamBackend::convertToPoseGraphMst(const std::set<StateId> & framesToConvert,
                                          const std::set<StateId> & framesToConsider) {
  // remember landmarks in frames (transformed to sensor frame)
  for(auto pose : framesToConvert) {
    ViGraph::State& state = realtimeGraph_.states_.at(pose);
    kinematics::Transformation T_SW = state.pose->estimate().inverse();
    MultiFramePtr mFrame = multiFrames_.at(pose);
    for(size_t i=0; i<mFrame->numFrames(); ++i) {
      for(size_t k=0; k<mFrame->numKeypoints(i); ++k) {
        uint64_t lmId = mFrame->landmarkId(i,k);
        if(lmId && realtimeGraph_.landmarkExists(LandmarkId(lmId))) {
          Eigen::Vector4d landmark = realtimeGraph_.landmark(LandmarkId(lmId));
          mFrame->setLandmark(i, k, T_SW*landmark,
                              realtimeGraph_.isLandmarkInitialised(LandmarkId(lmId)));
        }
      }
    }
  }

  std::vector<ViGraphEstimator::PoseGraphEdge> poseGraphEdges;
  std::vector<std::pair<StateId,StateId>> removedTwoPoseErrors;
  std::vector<KeypointIdentifier> removedObservations;
  realtimeGraph_.convertToPoseGraphMst(
        framesToConvert, framesToConsider, &poseGraphEdges, &removedTwoPoseErrors,
        &removedObservations);
  for(auto removedTwoPoseError : removedTwoPoseErrors) {
    observationLessGraph_.removeTwoPoseConstLink(
          removedTwoPoseError.first, removedTwoPoseError.second);
  }
  // clone edges to observationLess -- note the aliasing of the error term!!
  for(const auto & poseGraphEdge : poseGraphEdges) {
    observationLessGraph_.addExternalTwoPoseLink(
          poseGraphEdge.poseGraphError->cloneTwoPoseGraphErrorConst(),
          poseGraphEdge.referenceId, poseGraphEdge.otherId);

  }

  // also manage the full graph, if possible
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    for(const auto & poseGraphEdge : poseGraphEdges) {
      touchedStates_.insert(poseGraphEdge.referenceId);
      touchedStates_.insert(poseGraphEdge.otherId);
    }
    for(auto removedTwoPoseError : removedTwoPoseErrors) {
      touchedStates_.insert(removedTwoPoseError.first);
      touchedStates_.insert(removedTwoPoseError.second);
    }
    for(auto obs : removedObservations) {
      touchedStates_.insert(StateId(obs.frameId));
      if(fullGraph_.observations_.count(obs)) {
        LandmarkId lmId = fullGraph_.observations_.at(obs).landmarkId;
        if(lmId.isInitialised()) {
          touchedLandmarks_.insert(fullGraph_.observations_.at(obs).landmarkId); /// \todo hack, fix
        }
      }
    }
  } else {
    // replicate fullGraph_
    for(auto obs : removedObservations) {
      fullGraph_.removeObservation(obs);
    }
    for(auto removedTwoPoseError : removedTwoPoseErrors) {
      fullGraph_.removeTwoPoseConstLink(removedTwoPoseError.first, removedTwoPoseError.second);
    }
    for(const auto & poseGraphEdge : poseGraphEdges) {
      fullGraph_.addExternalTwoPoseLink(
            poseGraphEdge.poseGraphError->cloneTwoPoseGraphErrorConst(),
            poseGraphEdge.referenceId, poseGraphEdge.otherId);

    }
  }

  return true;
}

int ViSlamBackend::expandKeyframe(StateId keyframe)
{
  OKVIS_ASSERT_TRUE(Exception, imuFrames_.count(keyframe) == 0, "must be keyframe")
  OKVIS_ASSERT_TRUE(Exception, keyFrames_.count(keyframe) || loopClosureFrames_.count(keyframe),
                    "must be keyframe")
  OKVIS_ASSERT_TRUE(Exception, realtimeGraph_.states_.at(keyframe).twoPoseLinks.size()>0,
                    "must be frontier frame")
  int ctr = 0;
  std::set<LandmarkId> lms;
  std::set<StateId> cnncts;
  std::vector<ceres::TwoPoseGraphError::Observation> allObservations;
  realtimeGraph_.convertToObservations(keyframe, &lms, &cnncts, &allObservations);

  // also manage the full graph, if possible
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedStates_.insert(cnncts.begin(), cnncts.end());
    touchedLandmarks_.insert(lms.begin(), lms.end());
  } else {
    fullGraph_.removeTwoPoseConstLinks(keyframe);
    for(auto lm : lms) {
      const auto & landmark = realtimeGraph_.landmarks_.at(lm);
      if(!fullGraph_.landmarkExists(lm)) {
        fullGraph_.addLandmark(lm, landmark.hPoint->estimate(), landmark.hPoint->initialized());
        fullGraph_.setLandmarkQuality(lm, landmark.quality);
      }
    }
    for(const auto & obs : allObservations) {
      const bool useCauchy = obs.lossFunction != nullptr;
      LandmarkId landmarkId(obs.hPoint->id());
      fullGraph_.addExternalObservation(obs.reprojectionError, landmarkId,
                             obs.keypointIdentifier, useCauchy);
    }
  }

  observationLessGraph_.removeTwoPoseConstLinks(keyframe); // also remove; will re-add new ones
  for(auto cnnct : cnncts) {
    if(loopClosureFrames_.count(cnnct)==0 && auxiliaryStates_.at(cnnct).isPoseGraphFrame) {
      keyFrames_.insert(cnnct);
      auxiliaryStates_.at(cnnct).isPoseGraphFrame = false;
      ctr++;
    }
    if(keyFrames_.count(cnnct)==0 && auxiliaryStates_.at(cnnct).isPoseGraphFrame) {
      loopClosureFrames_.insert(cnnct);
      auxiliaryStates_.at(cnnct).isPoseGraphFrame = false;
      ctr++;
    }
  }

  return ctr;
}

void ViSlamBackend::eliminateImuFrames(size_t numImuFrames)
{
  std::set<StateId> imuFrames = imuFrames_; // copy so we can later delete
  for(auto id : imuFrames) {
    if(imuFrames_.size() <= numImuFrames) {
      break; // done -- reduced the IMU frames
    }
    if(realtimeGraph_.states_.at(id).isKeyframe) {
      // move to the keyframes that we keep
      imuFrames_.erase(id);
      keyFrames_.insert(id);
      components_.at(currentComponentIdx_).poseIds.insert(id);
      if(loopClosureFrames_.count(id)) {
        loopClosureFrames_.erase(id); // make sure the two sets are not intersecting
      }
      auxiliaryStates_.at(id).isImuFrame = false;
    } else {
      StateId refId = mostOverlappedStateId(id, false);
      auto observations = realtimeGraph_.states_.at(id).observations;
      realtimeGraph_.removeAllObservations(id);
      realtimeGraph_.eliminateStateByImuMerge(id, refId);

      observationLessGraph_.eliminateStateByImuMerge(id, refId);
      OKVIS_ASSERT_TRUE_DBG(Exception,
                        observationLessGraph_.states_.size()==realtimeGraph_.states_.size(),
                        "inconsistendy")
      // also remove from loop closure frames
      if(loopClosureFrames_.count(id)) {
        OKVIS_THROW(Exception, "bug")
      }

      // also manage the full graph, if possible
      if(isLoopClosing_ || isLoopClosureAvailable_) {
        for(const auto & obs : observations) {
          touchedLandmarks_.insert(obs.second.landmarkId);
        }
        eliminateStates_[id] = refId;
      } else {
        fullGraph_.removeAllObservations(id); /// \todo make more efficient (copy over)
        fullGraph_.eliminateStateByImuMerge(id, refId); /// \todo make more efficient (copy over)
      }

      imuFrames_.erase(id);
      multiFrames_.erase(id);
      auxiliaryStates_.erase(id);
    }
  }
}

bool ViSlamBackend::applyStrategy(size_t numKeyframes, size_t numLoopClosureFrames,
                                  size_t numImuFrames, bool expand)
{
  // check/handle lost
  TimerSwitchable t1("7.1 check/handle lost");
  if(imuFrames_.size()>1 && !keyFrames_.empty()) {
    auto iter = auxiliaryStates_.rbegin();
    if(trackingQuality(iter->first)<0.01) {
      LOG(INFO) << "TODO: handle lost/components. currently disabled.";
    }
    if(trackingQuality(iter->first)<0.00) { /// \todo: currently disabled. Re-introduce!
      // LOST! remember
      iter->second.isLost = true;

      // add an identity relative pose constraint (weak on orientation)
      Eigen::Matrix<double, 6,6> H = Eigen::Matrix<double, 6,6>::Identity();
      H(0,0) = 100.0; // 10 cm standard deviation
      H(1,1) = 100.0; // 10 cm standard deviation
      H(2,2) = 100.0; // 10 cm standard deviation
      kinematics::Transformation T_SS = kinematics::Transformation::Identity();
      StateId referenceId = *keyFrames_.rbegin(); // most recent KF as ref
      realtimeGraph_.addRelativePoseConstraint(iter->first, referenceId, T_SS, H);
      // also manage the full graph, if possible
      if(isLoopClosing_ || isLoopClosureAvailable_) {
        touchedStates_.insert(iter->first);
        touchedStates_.insert(referenceId);
      } else {
        fullGraph_.addRelativePoseConstraint(iter->first, referenceId, T_SS, H);
      }
      observationLessGraph_.addRelativePoseConstraint(iter->first, referenceId, T_SS, H);

      // remove previous constraint, if needed
      auto iterPreviousState = iter;
      iterPreviousState++;
      if(iterPreviousState->second.isLost && !iterPreviousState->second.isKeyframe) {
        auto relPoseLinks =
            realtimeGraph_.states_.at(iterPreviousState->first).relativePoseLinks;
        for(const auto & relPoseLink : relPoseLinks) {
          realtimeGraph_.removeRelativePoseConstraint(relPoseLink.first, iterPreviousState->first);
          observationLessGraph_.removeRelativePoseConstraint(
                relPoseLink.first, iterPreviousState->first);
          if(isLoopClosing_ || isLoopClosureAvailable_) {
            touchedStates_.insert(relPoseLink.first);
            touchedStates_.insert(iterPreviousState->first);
          } else {
            fullGraph_.removeRelativePoseConstraint(relPoseLink.first, iterPreviousState->first);
          }
        }
      }

      // check if a new component needs to start
      if(iterPreviousState->second.isLost && iter->second.isKeyframe) {
        if(iterPreviousState->second.isKeyframe) {
          // successive keyframes created, probably from bad detections
          iterPreviousState->second.isKeyframe = false;
        } else {
          // need to start a new component
          currentComponentIdx_ = components_.size();
          Component component;
          component.referenceId = iter->first;
          component.connectedReferenceId = referenceId;
          components_.push_back(component);
        }
      }
    }
  }
  t1.stop();

  // first eliminate the IMU frames that are too much and that are not keyframes
  TimerSwitchable t2("7.2 eliminate IMU frames");
  eliminateImuFrames(numImuFrames);
  t2.stop();

  // now tackle keyframes, if necessary
  StateId currentKeyframeId = currentKeyframeStateId();
  StateId currentFrameId = realtimeGraph_.currentStateId();

  if(!currentKeyframeId.isInitialised()) {
    return true; /// \todo fix this
  }

  // now eliminate keyframes
  TimerSwitchable t3("7.3 convert to posegraph");
  bool keyFrameEliminated = false;
  int ctrPg = 0;
  while(keyFrames_.size() > numKeyframes) {
    // find keyframe with least common observations with current frame or current keyframe
    realtimeGraph_.computeCovisibilities();
    currentKeyframeId = currentKeyframeStateId();
    currentFrameId = realtimeGraph_.currentStateId();
    StateId minId;
    int minObservations = 100000;
    for(auto keyFrame : keyFrames_) {
      const int coObs = std::max(realtimeGraph_.covisibilities(currentFrameId, keyFrame),
                              realtimeGraph_.covisibilities(currentKeyframeId, keyFrame));
      if(keyFrame == *keyFrames_.begin()){
        if(coObs>=2) {
          continue; // spare
        }
      }
      if(coObs < minObservations) {
        minObservations = coObs;
        minId = keyFrame;
      }
    }
    StateId maxId;
    int maxCoObs=0;
    std::set<StateId> framesToConsider;
    std::set<StateId> observedKeyframes = keyFrames_;
    observedKeyframes.insert(loopClosureFrames_.begin(), loopClosureFrames_.end());
    for(auto frameId : observedKeyframes) {
      const int coObs = realtimeGraph_.covisibilities(minId, frameId);
      if(coObs >= maxCoObs){
        maxId = frameId;
        maxCoObs = coObs;
      }
      if(realtimeGraph_.states_.at(frameId).twoPoseLinks.size()>0) {
        // this is a frontier node
        framesToConsider.insert(frameId);
      }
    }

    // convert it.
    std::set<StateId> convertToPosegraphFrames, allObservedFrames;
    convertToPosegraphFrames.insert(minId);
    auxiliaryStates_.at(minId).isPoseGraphFrame = true; // flag it to be posegraph frame
    keyFrames_.erase(minId);
    for(auto id : keyFrames_) {
      auxiliaryStates_.at(minId).recentLoopClosureFrames.insert(id); // don't re-add immediately.
    }
    framesToConsider.insert(minId);
    framesToConsider.insert(maxId);
    keyFrameEliminated = true;
    if(maxCoObs == 0) {
      // handle weird case that might happen with (almost) no matches
      realtimeGraph_.removeAllObservations(minId);
      // also manage the full graph, if possible
      if(isLoopClosing_ || isLoopClosureAvailable_) {
        touchedStates_.insert(minId);
      } else {
        fullGraph_.removeAllObservations(minId);
      }
      continue;
    }
    if(convertToPosegraphFrames.size() > 0) {
      bool success = convertToPoseGraphMst(convertToPosegraphFrames, framesToConsider);
      ctrPg++;
      OKVIS_ASSERT_TRUE(Exception, realtimeGraph_.states_.at(minId).observations.size()==0,
                        "terrible " << minId.value() << " " << int(success))
      if(ctrPg>=3) {
        break; // max 3 at the time...
      }
    }
  }
  t3.stop();

  // freeze old states
  TimerSwitchable t4("7.4 freezing");
  if(keyFrameEliminated) {
    auto iter = auxiliaryStates_.rbegin();
    StateId oldestKeyFrameId;
    for (size_t p = 0; p<(numKeyframes+numImuFrames) && iter!=auxiliaryStates_.rend(); ++p) {
      iter++;
    }
    if(iter!=auxiliaryStates_.rend()) {
      oldestKeyFrameId = iter->first;
    }

    int ctr = 0;
    for(auto iter = realtimeGraph_.states_.find(oldestKeyFrameId);
        iter != realtimeGraph_.states_.end(); --iter) {
      if(ctr==numRealtimePoseGraphFrames) {
        Time freezeTime = realtimeGraph_.states_.rbegin()->second.timestamp;
        while((freezeTime - iter->second.timestamp).toSec() < minDeltaT) {
          if(iter==realtimeGraph_.states_.begin()) {
            break;
          }
          --iter;
        }
        if(iter!=realtimeGraph_.states_.begin()) {
          // freezing of poses
          // make sure not to go and unfreeze old stuff
          // -- messes up synchronising if concurrentlz loop-optimising
          const StateId freezeId = std::max(lastFreeze_, iter->first);
          // freezing of poses
          lastFreeze_ = freezeId;
          if(realtimeGraph_.states_.find(freezeId) != realtimeGraph_.states_.begin()) {
            realtimeGraph_.freezePosesUntil(freezeId);
          }
          // freezing of speed/bias
          realtimeGraph_.freezeSpeedAndBiasesUntil(freezeId);
        }
        break;
      }
      if(iter==realtimeGraph_.states_.begin()) {
        break;
      }
      ctr++;
    }
  }
  t4.stop();

  // now tackle loop closure frames
  int ctrLc = 0;
  if(loopClosureFrames_.size() > 0) {
    TimerSwitchable t5("7.5 convert to posegraph");
    do {

      // find keyframe with least common observations with current frame or current keyframe
      realtimeGraph_.computeCovisibilities();
      currentKeyframeId = currentKeyframeStateId();
      currentFrameId = realtimeGraph_.currentStateId();
      StateId minId;
      int minObservations = 100000;
      for(auto loopClosureFrame : loopClosureFrames_) {
        const int coObs = std::max(realtimeGraph_.covisibilities(currentFrameId, loopClosureFrame),
                                realtimeGraph_.covisibilities(currentKeyframeId, loopClosureFrame));
        //size_t coObs = realtimeGraph_.covisibilities(currentFrameId, loopClosureFrame);
        if(coObs < minObservations) {
          minObservations = coObs;
          minId = loopClosureFrame;
        }
      }

      // eliminate no covisibility in any case
      if(minObservations!=0 && loopClosureFrames_.size()<=numLoopClosureFrames) {
        break;
      }

      std::set<StateId> framesToConsider;
      StateId maxId;
      int maxCoObs = 0;
      std::set<StateId> observedKeyframes = keyFrames_;
      observedKeyframes.insert(loopClosureFrames_.begin(), loopClosureFrames_.end());
      for(auto frameId : observedKeyframes) {
        const int coObs = realtimeGraph_.covisibilities(minId, frameId);
        if(coObs >= maxCoObs){
          maxId = frameId;
          maxCoObs = coObs;
        }
        if(realtimeGraph_.states_.at(frameId).twoPoseLinks.size()>0) {
          // this is a frontier node
          framesToConsider.insert(frameId);
        }
      }

      // convert it.
      std::set<StateId> convertToPosegraphFrames, allObservedFrames;
      convertToPosegraphFrames.insert(minId);
      auxiliaryStates_.at(minId).isPoseGraphFrame = true; // flag it to be posegraph frame
      loopClosureFrames_.erase(minId);
      framesToConsider.insert(minId);
      framesToConsider.insert(maxId);
      if(maxCoObs == 0) {
        // handle weird case that might happen with (almost) no matches
        realtimeGraph_.removeAllObservations(minId);
        // also manage the full graph, if possible
        if(isLoopClosing_ || isLoopClosureAvailable_) {
          touchedStates_.insert(minId);
        } else {
          fullGraph_.removeAllObservations(minId);
        }
        continue;
      }
      if(convertToPosegraphFrames.size() > 0 && maxCoObs > 0) {
        convertToPoseGraphMst(convertToPosegraphFrames, framesToConsider);
        ++ctrLc;
        if(ctrLc>=3) {
          break; // max 3 at the time.
        }
      }
    } while (loopClosureFrames_.size() > numLoopClosureFrames);
    t5.stop();
  }

  /// \todo the following would need re-optimisation of landmarks...
  // expand frontier, if necessary
  if(expand && ctrLc<3 && ctrPg<3) {
    TimerSwitchable t6("7.6 expand");
    currentKeyframeId = currentKeyframeStateId();
    if(currentKeyframeId.isInitialised()) {
      if(realtimeGraph_.states_.at(currentKeyframeId).twoPoseLinks.size()>0) {
        expandKeyframe(currentKeyframeId);
      }
    }
    const StateId currentLoopclosureFrameId = currentLoopclosureStateId();
    if(currentLoopclosureFrameId.isInitialised()) {
      if(realtimeGraph_.states_.at(currentLoopclosureFrameId).twoPoseLinks.size()>0) {
        expandKeyframe(currentLoopclosureFrameId);
      }
    }
    t6.stop();
  }

  // prune superfluous keyframes for future place recognition
  TimerSwitchable t7("7.7 prune place recognition frames");
  prunePlaceRecognitionFrames();
  t7.stop();

  return true;
}

void ViSlamBackend::optimiseRealtimeGraph(int numIter, std::vector<StateId> & updatedStates,
                                          int numThreads, bool verbose, bool onlyNewestState)
{
  // freeze if requested
  bool frozen = false;
  StateId unfreezeId;
  if(onlyNewestState) {
    // paranoid: find last frozen
    for(auto riter = realtimeGraph_.states_.rbegin(); riter != realtimeGraph_.states_.rend();
        ++riter) {
      if(riter->second.pose->fixed()) {
        break;
      } else {
        unfreezeId = riter->first;
      }
    }

    auto riter = realtimeGraph_.states_.rbegin();
    riter++;
    if(riter != realtimeGraph_.states_.rend()) {
      realtimeGraph_.freezePosesUntil(riter->first);
      realtimeGraph_.freezeSpeedAndBiasesUntil(riter->first);
      frozen = true;
    }
    for(const auto & lm : realtimeGraph_.landmarks_) {
      realtimeGraph_.problem_->SetParameterBlockConstant(lm.second.hPoint->parameters());
    }
  }



  // run the optimiser
  realtimeGraph_.options_.linear_solver_type = ::ceres::DENSE_SCHUR;
  realtimeGraph_.optimise(numIter, numThreads, verbose);

  // unfreeze if necessary
  if(onlyNewestState) {
    if(frozen) {
      realtimeGraph_.unfreezePosesFrom(unfreezeId);
      realtimeGraph_.unfreezeSpeedAndBiasesFrom(unfreezeId);
    }
    if(onlyNewestState) {
      for(const auto & lm : realtimeGraph_.landmarks_) {
        realtimeGraph_.problem_->SetParameterBlockVariable(lm.second.hPoint->parameters());
      }
    }
    return;
  }

  // import landmarks
  realtimeGraph_.updateLandmarks();

  // also copy states to observationless and fullGraph (if possible)
  for(auto riter = realtimeGraph_.states_.rbegin(); riter != realtimeGraph_.states_.rend();
      ++riter) {
    //if(riter == realtimeGraph_.states_.rbegin()) {
    //  std::cout << riter->first.value() << "#";
    //}
    if(riter->second.pose->fixed() && riter->second.speedAndBias->fixed()) {
      //std::cout << riter->first.value() << std::endl;
      break; // done, don't need to iterate further...
    }
    updatedStates.push_back(riter->first);
    ViGraph::State & observationLessState = observationLessGraph_.states_.at(riter->first);
    observationLessState.pose->setEstimate(riter->second.pose->estimate());
    observationLessState.speedAndBias->setEstimate(riter->second.speedAndBias->estimate());
    observationLessState.T_GW->setEstimate(riter->second.T_GW->estimate());
    /// consciously ignoring extrinsics here. \todo generalise for non-fixed extrinsics...
    if(!isLoopClosing_ && !isLoopClosureAvailable_) {
      ViGraph::State & fullState = fullGraph_.states_.at(riter->first);
      fullState.pose->setEstimate(riter->second.pose->estimate());
      fullState.speedAndBias->setEstimate(riter->second.speedAndBias->estimate());
      fullState.T_GW->setEstimate(riter->second.T_GW->estimate());
      //std::cout << "copying after realtime optimisation " << riter->first.value() << std::endl;
    }
  }

  // Check if GPS Trafo observable
  if(!gpsObservability_){ // gps not yet observable, copy estimate to other graphs if possible
      //  Copy T_GW estimates to observationLessGraph_ and fullGaph_
      observationLessGraph_.setGpsExtrinsics(realtimeGraph_.T_GW());
      // for fullGraph first check if possible -->  if(!isLoopClosing_ && !isLoopClosureAvailable_)
      if(!isLoopClosing_ && !isLoopClosureAvailable_){
          fullGraph_.setGpsExtrinsics(realtimeGraph_.T_GW());
      }
  }
  else{ // GPS is observable; check if fullGraph_ available and T_GW already fixed
      if(!realtimeGraph_.isGpsFixed()){ // realtimeGraph_ not yet fixed
          if(!isLoopClosing_ && !isLoopClosureAvailable_){ // fullGraph_ accessible
              realtimeGraph_.freezeGpsExtrinsics();
              std::cout << "Optimization leads to T_GW: \n" << realtimeGraph_.T_GW().T3x4() << std::endl;
              observationLessGraph_.setGpsExtrinsics(realtimeGraph_.T_GW());
              observationLessGraph_.freezeGpsExtrinsics();
              fullGraph_.setGpsExtrinsics(realtimeGraph_.T_GW());
              fullGraph_.freezeGpsExtrinsics();
          }
          // else : fullGraph_ not reachable, all Graphs will be fixed after fullGraph is optimised
          else{
              observationLessGraph_.setGpsExtrinsics(realtimeGraph_.T_GW());
          }
      }
  }

  // ... and landmarks to fullGraph (if possible)
  if(!isLoopClosing_ && !isLoopClosureAvailable_) {
    for(auto iter = realtimeGraph_.landmarks_.begin(); iter != realtimeGraph_.landmarks_.end();
        ++iter) {
      fullGraph_.setLandmark(iter->first, iter->second.hPoint->estimate(),
                             iter->second.hPoint->initialized());
      fullGraph_.setLandmarkQuality(iter->first, iter->second.quality);
    }
  }

  // finally adopt stuff from realtime optimisation results in full and observation-less graphs
  for(auto riter = realtimeGraph_.states_.rbegin(); riter != realtimeGraph_.states_.rend();
      ++riter) {
    if(!riter->second.previousImuLink.errorTerm) {
      continue;
    }
    observationLessGraph_.states_.at(riter->first).previousImuLink.errorTerm
        ->syncFrom(*riter->second.previousImuLink.errorTerm);
    if(!isLoopClosing_ && !isLoopClosureAvailable_) {
      fullGraph_.states_.at(riter->first).previousImuLink.errorTerm
          ->syncFrom(*riter->second.previousImuLink.errorTerm);
    }
    if(riter->second.pose->fixed()) {
      break;
    }
  }

  // check consistency -- currently disabled
}

bool ViSlamBackend::setOptimisationTimeLimit(double timeLimit, int minIterations)
{
  //fullGraph_.setOptimisationTimeLimit(timeLimit, 1); /// \todo Fix hack!
  return realtimeGraph_.setOptimisationTimeLimit(timeLimit, minIterations);
}

bool ViSlamBackend::isLandmarkAdded(LandmarkId landmarkId) const
{
  return realtimeGraph_.isLandmarkAdded(landmarkId);
}

bool ViSlamBackend::isLandmarkInitialised(LandmarkId landmarkId) const
{
  return realtimeGraph_.isLandmarkInitialised(landmarkId);
}

bool ViSlamBackend::setPose(StateId id, const kinematics::TransformationCacheless &pose)
{
  bool success = realtimeGraph_.setPose(id, pose);
  success &= observationLessGraph_.setPose(id, pose);
  // also manage the full graph, if possible
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedStates_.insert(id);
  } else {
    success &= fullGraph_.setPose(id, pose);
  }
  return success;
}

bool ViSlamBackend::setSpeedAndBias(StateId id, const SpeedAndBias &speedAndBias)
{
  bool success = realtimeGraph_.setSpeedAndBias(id, speedAndBias);
  success &= observationLessGraph_.setSpeedAndBias(id, speedAndBias);
  // also manage the full graph, if possible
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedStates_.insert(id);
  } else {
    success &= fullGraph_.setSpeedAndBias(id, speedAndBias);
  }
  return success;
}

bool ViSlamBackend::setExtrinsics(StateId id, uchar camIdx,
                                  const kinematics::TransformationCacheless & extrinsics)
{
  bool success = realtimeGraph_.setExtrinsics(id, camIdx, extrinsics);
  success &= observationLessGraph_.setExtrinsics(id, camIdx, extrinsics);
  // also manage the full graph, if possible
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    touchedStates_.insert(id);
  } else {
    success &= fullGraph_.setExtrinsics(id, camIdx, extrinsics);
  }
  return success;
}

bool ViSlamBackend::isInImuWindow(StateId id) const
{
  return auxiliaryStates_.at(id).isImuFrame;
}

Time ViSlamBackend::timestamp(StateId id) const
{
  return realtimeGraph_.timestamp(id);
}

void ViSlamBackend::drawOverheadImage(cv::Mat &image, int idx) const
{
  const ViGraphEstimator& graph = idx==0 ?
        realtimeGraph_ : (idx==1 ? observationLessGraph_ : fullGraph_);
  static const double borderPixels = 50;
  // first find max/min
  Eigen::Vector3d min(-0.05,-0.05,-0.05);
  Eigen::Vector3d max(0.05,0.05,0.05);
  for(auto iter=graph.states_.begin(); iter!=graph.states_.end(); ++iter) {
    const Eigen::Vector3d pos = iter->second.pose->estimate().r();
    for(int i=0; i<3; ++i) {
      if(pos[i] > max[i]) {
        max[i] = pos[i];
      }
      if(pos[i] < min[i]) {
        min[i] = pos[i];
      }
    }
  }
  const Eigen::Vector3d centre = 0.5*(max+min);
  const double scale = std::min((image.cols-2.0*borderPixels)/(max[0]-min[0]),
      (image.rows-2.0*borderPixels)/(max[1]-min[1]));

  // landmarks
  for(auto iter = graph.landmarks_.begin(); iter != graph.landmarks_.end(); ++iter) {
    const Eigen::Vector4d hp = iter->second.hPoint->estimate();
    const Eigen::Vector3d pos = (hp.head<3>()/hp[3]-centre)*scale;
    cv::Point2d cvPos(pos[0]+image.cols*0.5, -pos[1]+image.rows*0.5);
    const double quality = iter->second.quality; /// \todo save/retrieve real quality
    if(cvPos.x > 0.0 && cvPos.y > 0.0 && cvPos.x < (image.cols-1) && cvPos.y < (image.rows-1)) {
        cv::circle(image, cvPos, 1, cv::Scalar(0,std::min(255,20+int(quality/0.03*225.0)),0),
                                               cv::FILLED, cv::LINE_AA);
    }
  }

  // draw co-observations (need to re-compute as cannot call computeCovisibilities in const method)
  std::map<uint64_t, std::map<uint64_t, int>> coObservationCounts;
  for(auto iter=graph.landmarks_.begin(); iter!=graph.landmarks_.end(); ++iter) {
    auto obs = iter->second.observations;
    std::set<uint64> covisibilities;
    for(auto obsiter=obs.begin(); obsiter!=obs.end(); ++obsiter) {
      covisibilities.insert(obsiter->first.frameId);
    }
    for(auto i0=covisibilities.begin(); i0!=covisibilities.end(); ++i0) {
      for(auto i1=covisibilities.begin(); i1!=covisibilities.end(); ++i1) {
        if(*i1>=*i0) {
          continue;
        }
        if(coObservationCounts.find(*i0)==coObservationCounts.end()) {
          coObservationCounts[*i0][*i1] = 1;
        } else {
          if (coObservationCounts.at(*i0).find(*i1)==coObservationCounts.at(*i0).end()) {
            coObservationCounts.at(*i0)[*i1] = 1;
          } else {
            coObservationCounts.at(*i0).at(*i1)++;
          }
        }
      }
    }
  }

  for(auto i0=coObservationCounts.begin(); i0!=coObservationCounts.end(); ++i0) {
    for(auto i1=coObservationCounts.at(i0->first).begin();
        i1!=coObservationCounts.at(i0->first).end(); ++i1) {
      if(!graph.states_.count(StateId(i0->first))) continue;
      if(!graph.states_.count(StateId(i1->first))) continue;
      const Eigen::Vector3d p0 = graph.states_.at(StateId(i0->first)).pose->estimate().r();
      const Eigen::Vector3d p1 = graph.states_.at(StateId(i1->first)).pose->estimate().r();
      const Eigen::Vector3d pos0 = (p0-centre)*scale;
      const Eigen::Vector3d pos1 = (p1-centre)*scale;
      cv::Point2d cvPos0(pos0[0]+image.cols*0.5, -pos0[1]+image.rows*0.5);
      cv::Point2d cvPos1(pos1[0]+image.cols*0.5, -pos1[1]+image.rows*0.5);
      double brightness = 50.0 + std::min(205, i1->second*2);
      cv::line(image, cvPos0, cvPos1, cv::Scalar(0,brightness, brightness), 1, cv::LINE_AA);
    }
  }

  // draw old frames
  for(auto iter=graph.states_.begin(); iter!=graph.states_.end(); ++iter) {

    const Eigen::Vector3d pos = (iter->second.pose->estimate().r()-centre)*scale;
    cv::Point2d cvPos(pos[0]+image.cols*0.5, -pos[1]+image.rows*0.5);

    // draw pose graph error
    for(auto piter=iter->second.twoPoseLinks.begin();
        piter != iter->second.twoPoseLinks.end(); piter++) {
      auto refId = piter->second.errorTerm->referencePoseId();
      auto otherId = piter->second.errorTerm->otherPoseId();
      if(refId<otherId) {
        // draw only forward.
        const Eigen::Vector3d p0 = graph.states_.at(refId).pose->estimate().r();
        const Eigen::Vector3d p1 = graph.states_.at(otherId).pose->estimate().r();
        const Eigen::Vector3d pos0 = (p0-centre)*scale;
        const Eigen::Vector3d pos1 = (p1-centre)*scale;
        cv::Point2d cvPos0(pos0[0]+image.cols*0.5, -pos0[1]+image.rows*0.5);
        cv::Point2d cvPos1(pos1[0]+image.cols*0.5, -pos1[1]+image.rows*0.5);
        double brightness = 50.0 + std::min(205.0, piter->second.errorTerm->strength());
        //std::cout <<  piter->second.errorTerm->strength() << std::endl;
        cv::line(image, cvPos0, cvPos1, cv::Scalar(brightness/2,brightness, brightness), 3,
                 cv::LINE_AA);
      }
    }
    // draw pose graph error const
    for(auto piter=iter->second.twoPoseConstLinks.begin(); piter !=
        iter->second.twoPoseConstLinks.end(); piter++) {
      const Eigen::Vector3d p0 = graph.states_.at(iter->first).pose->estimate().r();
      const Eigen::Vector3d p1 = graph.states_.at(piter->first).pose->estimate().r();
      const Eigen::Vector3d pos0 = (p0-centre)*scale;
      const Eigen::Vector3d pos1 = (p1-centre)*scale;
      cv::Point2d cvPos0(pos0[0]+image.cols*0.5, -pos0[1]+image.rows*0.5);
      cv::Point2d cvPos1(pos1[0]+image.cols*0.5, -pos1[1]+image.rows*0.5);
      double brightness = 200;
      cv::line(image, cvPos0, cvPos1, cv::Scalar(brightness/2.0,brightness, brightness/2.0), 3,
               cv::LINE_AA);
    }
  }

  // draw frames
  for(auto iter=graph.states_.begin(); iter!=graph.states_.end(); ++iter) {

    const Eigen::Vector3d pos = (iter->second.pose->estimate().r()-centre)*scale;
    cv::Point2d cvPos(pos[0]+image.cols*0.5, -pos[1]+image.rows*0.5);
    auto colour1 = cv::Scalar(255,0,0);
    auto colour2 = cv::Scalar(255,255,255);
    if(!iter->second.isKeyframe) {
      colour1 = cv::Scalar(127,127,127);
    }
    if(idx<=2) {
        if(keyFrames_.count(iter->first)) {
          colour1 = cv::Scalar(255,255,0);
        }
        const bool isPoseGraphFrame = auxiliaryStates_.at(iter->first).isPoseGraphFrame;
        if(isPoseGraphFrame) {
          colour2 = cv::Scalar(127,127,127);
          if(iter->second.pose->fixed()) {
            colour2 = cv::Scalar(25,25,200);
          }
        }
    }

    // draw IMU error
    if(iter->second.nextImuLink.errorTerm) {
      auto iterNext = iter;
      iterNext++;
      const Eigen::Vector3d nextPos = (iterNext->second.pose->estimate().r()-centre)*scale;
      cv::Point2d nextCvPos(nextPos[0]+image.cols*0.5, -nextPos[1]+image.rows*0.5);
      cv::line(image, cvPos, nextCvPos, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }

    // draw point
    cv::circle(image, cvPos, 3, colour1, cv::FILLED, cv::LINE_AA);
    cv::circle(image, cvPos, 3, colour2, 1, cv::LINE_AA);
    std::stringstream stream;
    stream << iter->first.value();
    cv::putText(image, stream.str(), cvPos+cv::Point2d(6,3),
                cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255,255,255), 1, cv::LINE_AA);
  }

  if(idx >2) {
      // some text:
      auto T_WS = graph.states_.rbegin()->second.pose->estimate();
      const SpeedAndBias speedAndBias = graph.states_.rbegin()->second.speedAndBias->estimate();
      std::stringstream postext;
      postext << "position = ["
              << T_WS.r()[0] << ", " << T_WS.r()[1] << ", " << T_WS.r()[2] << "]";
      cv::putText(image, postext.str(), cv::Point(15,15),
                  cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
      std::stringstream veltext;
      veltext << "velocity = ["
              << speedAndBias[0] << ", " << speedAndBias[1] << ", " << speedAndBias[2] << "]";
      cv::putText(image, veltext.str(), cv::Point(15,35),
                      cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
      std::stringstream gyroBiasText;
      gyroBiasText << "gyro bias = ["
                   << speedAndBias[3] << ", " << speedAndBias[4] << ", " << speedAndBias[5] << "]";
      cv::putText(image, gyroBiasText.str(), cv::Point(15,55),
                      cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
      std::stringstream accBiasText;
      accBiasText << "acc bias = ["
                  << speedAndBias[6] << ", " << speedAndBias[7] << ", " << speedAndBias[8] << "]";
      cv::putText(image, accBiasText.str(), cv::Point(15,75),
                      cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
      return;
  }

  // check loopClosureFrame co-observability
  std::map<StateId, size_t> counts;
  for(auto iter = loopClosureFrames_.begin(); iter!=loopClosureFrames_.end(); ++iter) {
    counts[*iter] = 0;
  }
  const StateId cId = currentStateId();
  for (auto pit = graph.landmarks_.begin(); pit != graph.landmarks_.end(); ++pit) {
    const auto& residuals = pit->second.observations;
    bool isObservedInNewFrame = false;
    std::set<uint64_t> loopClosureFramesObserved;
    for (const auto& r : residuals) {
      uint64_t poseId = r.first.frameId;
      if(cId.value() == poseId) {
        isObservedInNewFrame = true;
      }
      if(counts.find(StateId(poseId)) != counts.end()) {
        loopClosureFramesObserved.insert(poseId);
      }
    }

    if(isObservedInNewFrame) {
      for(auto iter = loopClosureFramesObserved.begin(); iter!=loopClosureFramesObserved.end();
          ++iter) {
        counts.at(StateId(*iter))++;
      }
    }
  }

  // draw loop closure frames
  for(auto iter=loopClosureFrames_.begin(); iter!=loopClosureFrames_.end(); ++iter) {
    const Eigen::Vector3d pos = (graph.states_.at(*iter).pose->estimate().r()-centre)*scale;
    cv::Point2d cvPos(pos[0]+image.cols*0.5, -pos[1]+image.rows*0.5);
    auto colour1 = cv::Scalar(255,255,255);
    auto colour2 = cv::Scalar(255,0,0);

    // draw point
    cv::circle(image, cvPos, 5, colour1, cv::FILLED, cv::LINE_AA);
    cv::circle(image, cvPos, int(6.0f+float(counts.at(*iter))/10.0f), colour2,
               int(1+counts.at(*iter))/5, cv::LINE_AA);
  }

  // current position always on top
  auto T_WS = graph.states_.rbegin()->second.pose->estimate();
  const Eigen::Vector3d pos = (T_WS.r()-centre)*scale;
  cv::Point2d cvPos(pos[0]+image.cols*0.5, -pos[1]+image.rows*0.5);
  auto colour1 = cv::Scalar(127,127,127);
  auto colour2 = cv::Scalar(255,255,255);
  if(graph.states_.rbegin()->second.isKeyframe) {
    colour1 = cv::Scalar(255,255,0);
  }
  cv::circle(image, cvPos, 5, colour2, 2, cv::LINE_AA);
  cv::circle(image, cvPos, 3, colour1, cv::FILLED, cv::LINE_AA);

  // current keyframe
  StateId kfId = currentKeyframeStateId();
  if(kfId.isInitialised()) {
    kinematics::Transformation T_WS_kf = graph.states_.at(kfId).pose->estimate();
    const Eigen::Vector3d pos = (T_WS_kf.r()-centre)*scale;
    cv::Point2d cvPos(pos[0]+image.cols*0.5, -pos[1]+image.rows*0.5);
    auto colour1 = cv::Scalar(255,255,127);
    auto colour2 = cv::Scalar(255,255,255);

    cv::circle(image, cvPos, 5, colour2, 2, cv::LINE_AA);
    cv::circle(image, cvPos, 3, colour1, cv::FILLED, cv::LINE_AA);
  }

  // some text:
  const SpeedAndBias speedAndBias = graph.states_.rbegin()->second.speedAndBias->estimate();
  std::stringstream postext;
  postext << "position = [" << T_WS.r()[0] << ", " << T_WS.r()[1] << ", " << T_WS.r()[2] << "]";
  cv::putText(image, postext.str(), cv::Point(15,15),
              cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
  std::stringstream veltext;
  veltext << "velocity = ["
          << speedAndBias[0] << ", " << speedAndBias[1] << ", " << speedAndBias[2] << "]";
  cv::putText(image, veltext.str(), cv::Point(15,35),
                  cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
  std::stringstream gyroBiasText;
  gyroBiasText << "gyro bias = ["
               << speedAndBias[3] << ", " << speedAndBias[4] << ", " << speedAndBias[5] << "]";
  cv::putText(image, gyroBiasText.str(), cv::Point(15,55),
                  cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
  std::stringstream accBiasText;
  accBiasText << "acc bias = ["
              << speedAndBias[6] << ", " << speedAndBias[7] << ", " << speedAndBias[8] << "]";
  cv::putText(image, accBiasText.str(), cv::Point(15,75),
                  cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
}

bool ViSlamBackend::isLoopClosureFrame(StateId frameId) const {
  return loopClosureFrames_.count(frameId) > 0;
}

bool ViSlamBackend::isRecentLoopClosureFrame(StateId frameId) const {
  for(auto id : keyFrames_) {
    if(auxiliaryStates_.at(id).recentLoopClosureFrames.count(frameId)) {
      return true;
    }
  }
  for(auto id : loopClosureFrames_) {
    if(auxiliaryStates_.at(id).recentLoopClosureFrames.count(frameId)) {
      return true;
    }
  }
  return false;
}

void ViSlamBackend::addLoopClosureFrame(StateId loopClosureFrameId,
                                        std::set<LandmarkId> & loopClosureLandmarks,
                                        bool skipFullGraphOptimisation)
{
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_,
                    "trying to add loop closure while loop closing")
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosureAvailable_,
                    "trying to add loop closure while loop closing")

  // remember loop closure frame
  if(loopClosureFrames_.count(loopClosureFrameId)) {
    LOG(WARNING) << "trying to add previously added loop-closure frame";
    return;
  }
  if(keyFrames_.count(loopClosureFrameId)) {
    LOG(WARNING) << "trying to add current keyframe as loop-closure frame" << std::endl;
    return;
  }

  loopClosureFrames_.insert(loopClosureFrameId); // sort/remove later
  currentLoopClosureFrames_.insert(loopClosureFrameId);

  // undo pose graph errors into observations
  std::set<StateId> connectedStates;
  OKVIS_ASSERT_TRUE_DBG(Exception, auxiliaryStates_.at(loopClosureFrameId).isPoseGraphFrame,
                    "must be pose graph frame")
  std::vector<ceres::TwoPoseGraphError::Observation> allObservations;
  realtimeGraph_.convertToObservations(
        loopClosureFrameId, &loopClosureLandmarks, &connectedStates, &allObservations);
  for(auto lm : loopClosureLandmarks) {
    const auto & landmark = realtimeGraph_.landmarks_.at(lm);
    if(!fullGraph_.landmarkExists(lm)) {
      fullGraph_.addLandmark(lm, landmark.hPoint->estimate(), landmark.hPoint->initialized());
      fullGraph_.setLandmarkQuality(lm, landmark.quality);
    }
  }
  fullGraph_.removeTwoPoseConstLinks(loopClosureFrameId);
  for(const auto& obs : allObservations) {
    const bool useCauchy = obs.lossFunction != nullptr;
    LandmarkId landmarkId(obs.hPoint->id());
    fullGraph_.addExternalObservation(obs.reprojectionError, landmarkId,
                           obs.keypointIdentifier, useCauchy);
  }

  // also remove; will re-add new ones
  observationLessGraph_.removeTwoPoseConstLinks(loopClosureFrameId);

  // add also connected frames to loop closure frames
  // flag no longer pose graph frames
  for(auto connectedState : connectedStates) {
    if(auxiliaryStates_.at(connectedState).isPoseGraphFrame
       && keyFrames_.count(connectedState)==0) {
      loopClosureFrames_.insert(connectedState);
      currentLoopClosureFrames_.insert(connectedState);
      auxiliaryStates_.at(connectedState).isPoseGraphFrame = false;
    } // else already in active window and considered
  }
  auxiliaryStates_.at(loopClosureFrameId).isPoseGraphFrame = false;

  // remember oldest Id / freeze / unfreeze
  if(!loopClosureFrames_.empty()) {
    StateId oldestId = loopClosureFrameId;
    if(fullGraph_.states_.count(oldestId)) {
      StateId oldestIdToSetVariable = auxiliaryStates_.at(oldestId).loopId;
      // also remember this throughout the created loop
      for(auto iter = auxiliaryStates_.find(oldestIdToSetVariable);
          iter != auxiliaryStates_.end(); ++iter) {
        iter->second.loopId = oldestIdToSetVariable;
      }

      // apply the freeze/unfreeze
      Time oldestT = fullGraph_.states_.at(oldestId).timestamp;
      int ctr = 0;
      for(auto iter = auxiliaryStates_.find(oldestIdToSetVariable); ; --iter) {
        if(ctr == numPoseGraphFrames || iter==auxiliaryStates_.begin()) {
          while((oldestT - fullGraph_.timestamp(iter->first)).toSec()<minDeltaT) {
            if(iter==auxiliaryStates_.begin()) {
              break;
            }
            --iter;
          }
          fullGraph_.unfreezePosesFrom(iter->first);
          if(iter!=auxiliaryStates_.begin()) {
            fullGraph_.freezePosesUntil(iter->first);
          }
          fullGraph_.unfreezeSpeedAndBiasesFrom(iter->first);
          if(iter!=auxiliaryStates_.begin()) {
            fullGraph_.freezeSpeedAndBiasesUntil(iter->first);
          }
          break;
        }
        if(iter==auxiliaryStates_.begin()) {
          fullGraph_.unfreezePosesFrom(iter->first);
          fullGraph_.unfreezeSpeedAndBiasesFrom(iter->first);
          break;
        }
        ctr++;
      }
    }
  }

  // remember as recent loop closures
  for(auto id : keyFrames_) {
    auxiliaryStates_.at(id).recentLoopClosureFrames.insert(
          loopClosureFrames_.begin(), loopClosureFrames_.end());
  }
  for(auto id : loopClosureFrames_) {
    auxiliaryStates_.at(id).recentLoopClosureFrames.insert(
          loopClosureFrames_.begin(), loopClosureFrames_.end());
  }

  // signal we need optimisation
  if(!skipFullGraphOptimisation) {
    needsFullGraphOptimisation_ = true;
  }
}

void ViSlamBackend::addGpsAlignmentFrame(StateId gpsLossFrameId) {

  OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "trying to add gps loop closure while loop closing")
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosureAvailable_, "trying to add gps loop closure while loop closing")
  std::cout << "[DEBUG INFO GPS Alignment] Kicking off full graph optimisation due to gps loop closure, starting at state id = " << gpsLossFrameId.value()
            << " up to current state id  = " << fullGraph_.states_.rbegin()->first.value()<< std::endl;

  // remember oldest Id / freeze / unfreeze
  StateId oldestIdToSetVariable = gpsLossFrameId;

  Time oldestT = fullGraph_.states_.at(oldestIdToSetVariable).timestamp;
  int ctr = 0;

  //
  for(auto iter = fullGraph_.states_.find(oldestIdToSetVariable) ; ; --iter){
      if(ctr == numPoseGraphFrames || iter == fullGraph_.states_.begin()){
          while((oldestT - fullGraph_.timestamp(iter->first)).toSec()<minDeltaT) {
            if(iter==fullGraph_.states_.begin()) {
              break;
            }
            --iter;
          }
          fullGraph_.unfreezePosesFrom(iter->first);
          if(iter!=fullGraph_.states_.begin()) {
            fullGraph_.freezePosesUntil(iter->first);
          }
          fullGraph_.unfreezeSpeedAndBiasesFrom(iter->first);
          if(iter!=fullGraph_.states_.begin()) {
            fullGraph_.freezeSpeedAndBiasesUntil(iter->first);
          }
          break;
      }
      if(iter==fullGraph_.states_.begin()) {
        fullGraph_.unfreezePosesFrom(iter->first);
        fullGraph_.unfreezeSpeedAndBiasesFrom(iter->first);
        break;
      }
      ctr++; // count states that will be optimised during global alignment
  }

  // signal we need optimisation
  needsFullGraphOptimisation_ = true;
  std::cout << "[DEBUG INFO GPS Alignment] Triggering full graph optimisation." << std::endl;
}

bool ViSlamBackend::synchroniseRealtimeAndFullGraph(std::vector<StateId> &updatedStates)
{
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "cannot synchronise while loop-closing")

  // first, we move the loop-closure frames into the set of regular keyframes...
  keyFrames_.insert(loopClosureFrames_.begin(), loopClosureFrames_.end());
  loopClosureFrames_.clear();

  // remove landmarks in full graph that have disappeared
  auto landmarks = fullGraph_.landmarks_;
  for(auto iter = landmarks.begin(); iter != landmarks.end(); ++iter) {
    if(realtimeGraph_.landmarks_.count(iter->first) == 0) {
      fullGraph_.removeLandmark(iter->first);
    }
  }

  // compute pose change
  StateId oldFrameId;
  for(auto iter = fullGraph_.states_.crbegin(); iter != fullGraph_.states_.crend(); ++iter) {
    if(realtimeGraph_.states_.count(iter->first)) {
      oldFrameId = iter->first;
      break;
    }
  }

  const kinematics::Transformation T_WS_old = realtimeGraph_.pose(oldFrameId);
  const kinematics::Transformation T_WS_new = fullGraph_.pose(oldFrameId);
  const kinematics::Transformation T_Wnew_Wold = T_WS_new * T_WS_old.inverse();

  // process added new states
  for(const auto& addState : addStatesBacklog_) {
    bool asKeyframe = false;
    const bool existsInRealtimeGraph = realtimeGraph_.states_.count(addState.id) !=0;
    if(existsInRealtimeGraph) {
      if(realtimeGraph_.states_.at(addState.id).isKeyframe) {
        asKeyframe = true;
      }
    }
    fullGraph_.addStatesPropagate(addState.timestamp, addState.imuMeasurements, asKeyframe);
    kinematics::Transformation T_WS;
    SpeedAndBias speedAndBias;
    if(!existsInRealtimeGraph) {
      const auto& anystate = realtimeGraph_.anyState_.at(addState.id);
      const kinematics::Transformation T_Sk_S = anystate.T_Sk_S;
      const kinematics::Transformation T_WSk =
          T_Wnew_Wold*realtimeGraph_.states_.at(anystate.keyframeId).pose->estimate();
      T_WS = T_WSk*T_Sk_S;
      speedAndBias = fullGraph_.states_.at(addState.id).speedAndBias->estimate();
      speedAndBias.head<3>() = T_WSk.C() * anystate.v_Sk;
    } else {
      const kinematics::Transformation T_S0S1 =
          realtimeGraph_.states_.at(oldFrameId).pose->estimate().inverse()*
          realtimeGraph_.states_.at(addState.id).pose->estimate();
      T_WS = fullGraph_.states_.at(oldFrameId).pose->estimate()*T_S0S1;
      const SpeedAndBias speedAndBias_old =
          realtimeGraph_.states_.at(addState.id).speedAndBias->estimate();
      speedAndBias = fullGraph_.states_.at(addState.id).speedAndBias->estimate();
      const Eigen::Vector3d v_S =
          realtimeGraph_.states_.at(oldFrameId).pose->estimate().inverse().C()
          *speedAndBias_old.head<3>();
      speedAndBias.head<3>() = T_WS.C()*v_S;
    }
    fullGraph_.setPose(addState.id, T_WS);
    fullGraph_.setSpeedAndBias(addState.id, speedAndBias);
  }
  addStatesBacklog_.clear();

  for(const auto& eliminateState : eliminateStates_) {
    fullGraph_.removeAllObservations(eliminateState.first);
    /// \todo make more efficient (copy over)
    fullGraph_.eliminateStateByImuMerge(eliminateState.first, eliminateState.second);
    /// \todo make more efficient (copy over)
  }
  eliminateStates_.clear();

  for(const auto& eliminateState : eliminateStates_) {
    const auto iter = fullGraph_.states_.find(eliminateState.second);
    if(iter != fullGraph_.states_.end()) {
      if(iter->second.previousImuLink.errorTerm) {
        iter->second.previousImuLink.errorTerm
            ->syncFrom(*realtimeGraph_.states_.at(eliminateState.second).previousImuLink.errorTerm);
      }
    }
  }
  for(auto riter = fullGraph_.states_.crbegin(); riter != fullGraph_.states_.crend(); ++riter) {
    if(riter->second.pose->fixed() && riter->second.speedAndBias->fixed()
         && realtimeGraph_.states_.at(riter->first).pose->fixed()
         && realtimeGraph_.states_.at(riter->first).speedAndBias->fixed()
         && observationLessGraph_.states_.at(riter->first).pose->fixed()
         && observationLessGraph_.states_.at(riter->first).speedAndBias->fixed()
       ) {
      break;
    }
    auto & errorTerm = realtimeGraph_.states_.at(riter->first).previousImuLink.errorTerm;
    if(errorTerm) {
      errorTerm->syncFrom(*riter->second.previousImuLink.errorTerm);
    }
  }

  // update new landmarks with pose change and insert into full graph
  for(auto iter = realtimeGraph_.landmarks_.begin(); iter != realtimeGraph_.landmarks_.end();
      ++iter) {
    if(fullGraph_.landmarks_.count(iter->first) == 0) {
      realtimeGraph_.setLandmark(iter->first, T_Wnew_Wold * iter->second.hPoint->estimate());
      fullGraph_.addLandmark(
            iter->first, iter->second.hPoint->estimate(), iter->second.hPoint->initialized());
      fullGraph_.setLandmarkQuality(iter->first, iter->second.quality);
    }
  }

  // ----- gps stuff begin -----
  std::cout << "Most current state: " << realtimeGraph_.states_.rbegin()->first.value() << std::endl;
  std::cout << "Finished fullGraph_ optimisation. starting to add buffered measurements" << std::endl;
  // Process buffered gps measurements
  for(auto addGpsMeas : addGpsBacklog_){
      bool stillExistsInRealtimeGraph = fullGraph_.states_.count(addGpsMeas.id) !=0;
      if(stillExistsInRealtimeGraph) {
          if(addGpsMeas.reInitFlag){
              std::cout << "reinit fullGraph_" << std::endl;
            fullGraph_.reInitGpsExtrinsics();
            }
          std::cout << "reversely adding measurements to fullGraph at state " << addGpsMeas.id.value() << std::endl;
          fullGraph_.addGpsMeasurement(addGpsMeas.id, addGpsMeas.gpsMeasurement, addGpsMeas.imuMeasurements);
      }
  }
  addGpsBacklog_.clear();
  // only have to do sth if fullGraph_.T_GW has not yet been fixed
  if(!fullGraph_.isGpsFixed()){
      const kinematics::Transformation T_GW_new = fullGraph_.T_GW(); // fullGraph Optimisation Result
      // copy results
      realtimeGraph_.setGpsExtrinsics(T_GW_new);
      observationLessGraph_.setGpsExtrinsics(T_GW_new);
      if(gpsObservability_){ // copy results and FIX
          std::cout << "[DEBUG ViSlamBackend] Fixing T_GW from fullGraph_ optimisation." << std::endl;
          realtimeGraph_.setGpsExtrinsics(T_GW_new);
          observationLessGraph_.setGpsExtrinsics(T_GW_new);
          fullGraph_.freezeGpsExtrinsics();
          observationLessGraph_.freezeGpsExtrinsics();
          realtimeGraph_.freezeGpsExtrinsics();
      }
  }
  // ----- gps stuff end -----

  // copy the result over now
  for(auto riter = fullGraph_.states_.crbegin(); riter != fullGraph_.states_.crend(); ++riter) {
    if(riter->second.pose->fixed() && riter->second.speedAndBias->fixed()
         && realtimeGraph_.states_.at(riter->first).pose->fixed()
         && realtimeGraph_.states_.at(riter->first).speedAndBias->fixed()
         && observationLessGraph_.states_.at(riter->first).pose->fixed()
         && observationLessGraph_.states_.at(riter->first).speedAndBias->fixed()
       ) {
      OKVIS_ASSERT_TRUE(Exception, riter->second.pose->estimate().T()
                        ==realtimeGraph_.pose(riter->first).T(), "O-O")
      OKVIS_ASSERT_TRUE(Exception, riter->second.pose->estimate().T()
                        ==observationLessGraph_.pose(riter->first).T(), "O-O")
      OKVIS_ASSERT_TRUE(Exception, riter->second.speedAndBias->estimate()
                        ==realtimeGraph_.speedAndBias(riter->first), "O-O")
      OKVIS_ASSERT_TRUE(Exception, riter->second.speedAndBias->estimate()
                        ==observationLessGraph_.speedAndBias(riter->first), "O-O")
      break;
    } else {
      updatedStates.push_back(riter->first);
    }
    if(realtimeGraph_.states_.count(riter->first) == 0) {
      OKVIS_THROW(Exception, "impossible")
      // new state not yet added
      continue;
    }
    realtimeGraph_.setPose(riter->first, riter->second.pose->estimate());
    observationLessGraph_.setPose(riter->first, riter->second.pose->estimate());
    realtimeGraph_.setSpeedAndBias(riter->first, riter->second.speedAndBias->estimate());
    observationLessGraph_.setSpeedAndBias(riter->first, riter->second.speedAndBias->estimate());
    for(size_t i = 0; i<riter->second.extrinsics.size(); ++i) {
      if(!riter->second.extrinsics.at(i)->fixed()) {
        realtimeGraph_.setExtrinsics(riter->first, uchar(i),
                                     riter->second.extrinsics.at(i)->estimate());
        observationLessGraph_.setExtrinsics(riter->first, uchar(i),
                                 riter->second.extrinsics.at(i)->estimate());
      }
    }
  }

  // update landmarks
  for(auto iter = fullGraph_.landmarks_.begin(); iter != fullGraph_.landmarks_.end(); ++iter) {
    OKVIS_ASSERT_TRUE_DBG(Exception, realtimeGraph_.landmarkExists(iter->first), "not allowed")
    realtimeGraph_.setLandmark(iter->first, iter->second.hPoint->estimate(),
                               iter->second.hPoint->initialized());
    realtimeGraph_.setLandmarkQuality(iter->first, iter->second.quality);
  }

  // process touched landmarks/observations
  OKVIS_ASSERT_TRUE(Exception, fullGraph_.landmarks_.size() == realtimeGraph_.landmarks_.size(),
                    "inconsistent")
  for(auto lm : touchedLandmarks_) {
    if(!fullGraph_.landmarkExists(lm)) {
      continue;
    }
    // remove all
    auto observations = fullGraph_.landmarks_.at(lm).observations;
    for(const auto & obs : observations) {
      fullGraph_.removeObservation(obs.first);
    }
  }
  for(auto lm : touchedLandmarks_) {
    if(!fullGraph_.landmarkExists(lm)) {
      continue;
    }
    // copy over
    for(const auto & obs : realtimeGraph_.landmarks_.at(lm).observations) {
      fullGraph_.addExternalObservation(obs.second.errorTerm, lm, obs.first, true);
    }
  }

  // remove all other edges
  for(auto stateId : touchedStates_) {
    if(fullGraph_.states_.count(stateId) == 0) {
      continue; // later deleted
    }
    auto relativePoselinks = fullGraph_.states_.at(stateId).relativePoseLinks;
    for(const auto& link : relativePoselinks) {
      fullGraph_.removeRelativePoseConstraint(link.second.state0, link.second.state1);
    }
    auto twoPoselinks = fullGraph_.states_.at(stateId).twoPoseConstLinks;
    for(const auto& link : twoPoselinks) {
      fullGraph_.removeTwoPoseConstLink(link.second.state0, link.second.state1);
    }
  }

  // re-add
  std::set<::ceres::ResidualBlockId> addedRelativePoseLinks;
  std::set<::ceres::ResidualBlockId> addedTwoPoseLinks;
  for(auto stateId : touchedStates_) {
    if(fullGraph_.states_.count(stateId) == 0) {
      continue; // later deleted
    }
    for(const auto& relPoseLink : realtimeGraph_.states_.at(stateId).relativePoseLinks) {
      if(addedRelativePoseLinks.count(relPoseLink.second.residualBlockId)==0) {
        fullGraph_.addRelativePoseConstraint(
              relPoseLink.second.state0, relPoseLink.second.state1,
              relPoseLink.second.errorTerm->T_AB(),
              relPoseLink.second.errorTerm->information());
        addedRelativePoseLinks.insert(relPoseLink.second.residualBlockId);
      }
    }
    for(const auto& twoPoseLink : realtimeGraph_.states_.at(stateId).twoPoseLinks) {
      if(addedTwoPoseLinks.count(twoPoseLink.second.residualBlockId)==0) {
        fullGraph_.addExternalTwoPoseLink(
              twoPoseLink.second.errorTerm->cloneTwoPoseGraphErrorConst(),
              twoPoseLink.second.state0, twoPoseLink.second.state1);
        addedTwoPoseLinks.insert(twoPoseLink.second.residualBlockId);
      }
    }
  }
  touchedStates_.clear();
  touchedLandmarks_.clear();

  OKVIS_ASSERT_TRUE(Exception, fullGraph_.landmarks_.size() == realtimeGraph_.landmarks_.size(),
                    "inconsistent")
  OKVIS_ASSERT_TRUE(Exception, fullGraph_.states_.size() == realtimeGraph_.states_.size(),
                    "inconsistent")

  // loop closure processed
  currentLoopClosureFrames_.clear();
  isLoopClosureAvailable_ = false;

  return true;
}

int ViSlamBackend::cleanUnobservedLandmarks() {
  std::map<LandmarkId, std::set<KeypointIdentifier>> removed;
  int removed1 = realtimeGraph_.cleanUnobservedLandmarks(&removed);
  for(const auto & rem : removed) {
    for(const auto & obs : rem.second) {
      // note: it can happen (rarely) that a landmark gets cleaned but then re-added,
      // so the respective frame observations might be missed in the synchronisation.
      multiFrame(StateId(obs.frameId))->setLandmarkId(obs.cameraIndex, obs.keypointIndex, 0);
    }
  }
  if(isLoopClosing_ || isLoopClosureAvailable_) {
    for(const auto & rem : removed) {
      touchedLandmarks_.insert(rem.first);
      for(auto obs : rem.second) {
        // note: it can happen (rarely) that a landmark gets cleaned but then re-added,
        // so the respective frame observations might be missed in the synchronisation.
        touchedStates_.insert(StateId(obs.frameId));
      }
    }
    return removed1; /// \todo This can be done with some refactoring
  } else {
    int removed0 = fullGraph_.cleanUnobservedLandmarks();
    OKVIS_ASSERT_TRUE(Exception, removed0 == removed1, "aha")
    return removed1;
  }

}

int ViSlamBackend::mergeLandmarks(std::vector<LandmarkId> fromIds, std::vector<LandmarkId> intoIds)
{
  OKVIS_ASSERT_TRUE_DBG(Exception, fromIds.size() == intoIds.size(), "vectors must be same lengths")
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "loop closure not finished, cannot merge landmarks")
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosureAvailable_,
                    "loop closure not finished, cannot merge landmarks")

  std::map<LandmarkId, LandmarkId> changes; // keep track of changes
  int ctr = 0;
  for(size_t i = 0; i < fromIds.size(); ++i) {
    // check if the fromId hasn't already been changed
    while(changes.count(fromIds.at(i))) {
      fromIds.at(i) = changes.at(fromIds.at(i));
    }
    // check if the intoId hasn't already been changed
    while(changes.count(intoIds.at(i))) {
      intoIds.at(i) = changes.at(intoIds.at(i));
    }
    // check if the change hasn't been indirectly applied already
    if(fromIds.at(i) == intoIds.at(i)) {
      continue; //this has already been done.
    }

    // now merge
    if(realtimeGraph_.mergeLandmark(fromIds.at(i), intoIds.at(i), multiFrames_)) {
      ctr++;
    }
    fullGraph_.mergeLandmark(fromIds.at(i), intoIds.at(i), multiFrames_);
    changes[fromIds.at(i)] = intoIds.at(i);

    // also reset associated keypoints
    auto observations = realtimeGraph_.landmarks_.at(intoIds.at(i)).observations;
    for(const auto & observation : observations) {
      multiFrames_.at(StateId(observation.first.frameId))->setLandmarkId(
            observation.first.cameraIndex, observation.first.keypointIndex, intoIds.at(i).value());
    }
  }

  return ctr;
}

void ViSlamBackend::optimiseFullGraph(int numIter, ::ceres::Solver::Summary &summary,
                                      int numThreads, bool verbose)
{
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "can't optimise posegraph; already loop-closing")
  std::cout << "Starting fullGraph_ optimisation." << std::endl;
  needsFullGraphOptimisation_ = false;
  isLoopClosing_ = true;
  kinematics::Transformation T_SS_measured;
  StateId pose_i, pose_j;
  for(auto & item : fullGraphRelativePoseConstraints_) {
    T_SS_measured = item.T_Si_Sj;
    pose_i = item.pose_i;
    pose_j = item.pose_j;
    fullGraph_.addRelativePoseConstraint(item.pose_i, item.pose_j, item.T_Si_Sj,
                                         100*item.information);
  }
  fullGraph_.options().function_tolerance = 0.001;
  fullGraph_.optimise(numIter*3, numThreads, verbose);
  for(auto & item : fullGraphRelativePoseConstraints_) {
    fullGraph_.removeRelativePoseConstraint(item.pose_i, item.pose_j);
  }
  fullGraphRelativePoseConstraints_.clear();
  fullGraph_.options().function_tolerance = 1e-6;

  fullGraph_.optimise(numIter-numIter/3, numThreads, verbose);
  summary = fullGraph_.summary();
  fullGraph_.updateLandmarks(); /// \todo check to do better

  isLoopClosureAvailable_ = true;
  isLoopClosing_ = false;
}

void ViSlamBackend::doFinalBa(
    int numIter, ::ceres::Solver::Summary &summary, double extrinsicsPositionUncertainty,
    double extrinsicsOrientationUncertainty, int numThreads, bool verbose)
{
  // convert all posegraph edges
  for(const auto & state : fullGraph_.states_) {
    if(state.second.twoPoseConstLinks.size()>0) {
      if(keyFrames_.count(state.first) == 0 && loopClosureFrames_.count(state.first) == 0) {
        keyFrames_.insert(state.first);
      }
      expandKeyframe(state.first);
    }
  }

  // unfreeze
  fullGraph_.unfreezePosesFrom(StateId(1));
  fullGraph_.unfreezeSpeedAndBiasesFrom(StateId(1));

  // make sure IMU errors are reintegrated if needed
  ceres::ImuError::redoPropagationAlways = true;

  // optimise
  optimiseFullGraph(numIter, summary,numThreads, verbose);

  // remove speed and bias prior
  fullGraph_.removeSpeedAndBiasPrior(StateId(1));

  // remove extrinsics fixation
  if(extrinsicsPositionUncertainty > 0.0 && extrinsicsOrientationUncertainty > 0.0) {
    fullGraph_.setExtrinsicsVariable();
    fullGraph_.softConstrainExtrinsics(
          extrinsicsPositionUncertainty, extrinsicsOrientationUncertainty);
  }

  // optimise
  optimiseFullGraph(numIter, summary,numThreads, verbose);

  cv::Mat img=cv::Mat::zeros(2000,2000,CV_8UC3);
  drawOverheadImage(img, 2);
  cv::imwrite("fullBa.png", img);

  // print
  const auto & state = fullGraph_.states_.rbegin()->second;
  for(size_t i = 0; i < state.extrinsics.size(); ++i) {
    std::cout << "extrinsics T_SC" << i << ":" << std::endl;
    std::cout << std::setprecision(15) << state.extrinsics.at(i)->estimate().T() << std::endl;
  }
  Eigen::Matrix<double,6,1> biases = Eigen::Matrix<double,6,1>::Zero();
  for(const auto & s : fullGraph_.states_) {
    biases += s.second.speedAndBias->estimate().tail<6>();
  }
  biases = (1.0/double(fullGraph_.states_.size()))*biases;
  std::cout << "biases: " << std::setprecision(15) << biases.transpose() << std::endl;
  Eigen::Matrix<double,6,1> biasesStd = Eigen::Matrix<double,6,1>::Zero();
  for(const auto & s : fullGraph_.states_) {
    Eigen::Matrix<double,6,1> diff = (s.second.speedAndBias->estimate().tail<6>()-biases);
    biasesStd += diff.cwiseAbs2();
  }
  std::cout << "biases stdev: " << biasesStd.cwiseSqrt().transpose() << std::endl;

  // Prepare histogram plotting: < -5 | -5 < -4 | ... | -1 < 0 | 0 < 1 | ... | > 5 => 12 bins
  // index: floor(x) + 6
  // save per camera
  std::vector<Eigen::Matrix<int, 12, 1>, Eigen::aligned_allocator<Eigen::Matrix<int, 12, 1>>> errorHistograms;
  for(size_t im = 0; im<multiFrames_.rbegin()->second->numFrames(); ++im) {
    Eigen::Matrix<int, 12, 1> errHist;
    errHist.setZero();
    errorHistograms.push_back(errHist);
  }

  // some plotting (currently disabled)
  std::vector<cv::Mat> images;
  std::vector<cv::Mat> outliers;
  std::vector<int> inlierCtrs;
  std::vector<int> outlierCtrs;
  for(size_t im = 0; im<multiFrames_.rbegin()->second->numFrames(); ++im) {
    inlierCtrs.push_back(0);
    outlierCtrs.push_back(0);
    images.push_back(cv::Mat::zeros(
          int(multiFrames_.rbegin()->second->geometry(im)->imageHeight()),
          int(multiFrames_.rbegin()->second->geometry(im)->imageWidth()), CV_8UC3));
    outliers.push_back(cv::Mat::zeros(
          int(multiFrames_.rbegin()->second->geometry(im)->imageHeight()),
          int(multiFrames_.rbegin()->second->geometry(im)->imageWidth()), CV_8UC3));
  }

  for(const auto & obs : fullGraph_.observations_) {
    auto frame = multiFrames_.at(StateId(obs.first.frameId));
    cv::KeyPoint kpt;
    frame->getCvKeypoint(obs.first.cameraIndex, obs.first.keypointIndex, kpt);
    double* params[3];
    params[0] = fullGraph_.states_.at(StateId(obs.first.frameId)).pose->parameters();
    params[1] = fullGraph_.landmarks_.at(obs.second.landmarkId).hPoint->parameters();
    params[2] = fullGraph_.states_.at(StateId(obs.first.frameId)).extrinsics.at(
          obs.first.cameraIndex)->parameters();
    Eigen::Vector2d err;
    obs.second.errorTerm->Evaluate(params, err.data(), nullptr);
    double badness = sqrt(err.transpose()*err)/3.0; // Chi2 threshold 9
    if(badness>1.0) {
      cv::circle(outliers.at(obs.first.cameraIndex), kpt.pt, 1, cv::Scalar(255,0,0),
                 cv::FILLED, cv::LINE_AA);
      outlierCtrs.at(obs.first.cameraIndex)++;
    } else {
      cv::Scalar colour(0,std::max(0.0,255.0*(1.0-badness)),std::min(255.0,255.0*badness));
      cv::circle(images.at(obs.first.cameraIndex), kpt.pt, 1, colour, cv::FILLED, cv::LINE_AA);
      inlierCtrs.at(obs.first.cameraIndex)++;
    }

    // histogram filling
    double errNorm = err.norm();
    int idx = static_cast<int> (std::floor(errNorm)) + 6;
    if(idx > 11)
      idx = 11;
    else if(idx < 0)
      idx = 0;
//    if(errNorm < 0.)
//      std::cout << "WEIRD! " << std::endl;
//    if(idx > 0) {
//      std::cout << "error : " << err.transpose() << " with norm " << err.norm() << std::endl;
//      std::cout << "     resulting idx : " << idx << " because of floor  " << std::floor(err.norm()) << std::endl;
//    }
    errorHistograms.at(obs.first.cameraIndex)[idx] += 1;

  }

  for(size_t im = 0; im<multiFrames_.rbegin()->second->numFrames(); ++im) {
    //std::stringstream namestr;
    //namestr << "Reprojection error image " << im << " (" << inlierCtrs.at(im) << ")";
    //cv::imshow(namestr.str(), images[im]);
    //std::stringstream outlierstr;
    //outlierstr << "Outliers image " << im << " (" << outlierCtrs.at(im) << ")";
    //cv::imshow(outlierstr.str(), outliers[im]);
    // histogram printing
    std::cout << "Cam " << im << " reprojection error histogram: " << errorHistograms.at(im).transpose() << std::endl;
  }
}

void ViSlamBackend::getFinalStateList(std::vector<StateId> &fullGraphStateIds){

  // Iter all States in fullgraph
  for(const auto & state : fullGraph_.states_){
    fullGraphStateIds.push_back(state.first);
  }
}

bool ViSlamBackend::saveMap(std::string path)
{
  std::ofstream file(path);
  if(!file.good()) {
    return false;
  }
  for(auto & landmark : realtimeGraph_.landmarks_) {
    if(landmark.second.quality > 0.001) {
      // save position
      Eigen::Vector4d hposition = landmark.second.hPoint->estimate();
      Eigen::Vector3d position = hposition.head<3>()/hposition[3];
      file << position[0] << "," << position[1] << "," << position[2];
      // save descriptors
      for(auto & obs : landmark.second.observations) {
        file << ",";
        // retrieve descriptor
        const unsigned char* descriptor =
            multiFrames_.at(StateId(obs.first.frameId))->keypointDescriptor(
              obs.first.cameraIndex, obs.first.keypointIndex);
        for(size_t i=0; i<48; ++i) {
          file << std::setfill('0') << std::setw(2) << std::hex << uint32_t(descriptor[i]);
        }
      }
      file << std::endl;
    }
  }

  return true;
}

bool ViSlamBackend::writeFinalCsvTrajectory(const std::string &csvFileName, bool rpg) const {
  std::fstream csvFile(csvFileName.c_str(), std::ios_base::out);
  bool success = csvFile.good();
  if (!success) {
    return false;
  }

  // write description
  if (rpg) {
    csvFile << "# timestamp tx ty tz qx qy qz qw" << std::endl;
  } else {
    csvFile << "timestamp" << ", " << "p_WS_W_x" << ", " << "p_WS_W_y" << ", "
            << "p_WS_W_z" << ", " << "q_WS_x" << ", " << "q_WS_y" << ", "
            << "q_WS_z" << ", " << "q_WS_w" << ", " << "v_WS_W_x" << ", "
            << "v_WS_W_y" << ", " << "v_WS_W_z" << ", " << "b_g_x" << ", "
            << "b_g_y" << ", " << "b_g_z" << ", " << "b_a_x" << ", " << "b_a_y"
            << ", " << "b_a_z" << ", " << "NrGps" << ", " << "SID" << ", " << "gpsMode" << std::endl;
    for (auto iter = realtimeGraph_.anyState_.begin(); iter != realtimeGraph_.anyState_.end(); ++iter) {
      Eigen::Vector3d p_WS_W;
      Eigen::Quaterniond q_WS;
      SpeedAndBias speedAndBiases;
      std::stringstream time;
      size_t nrGps;
      size_t gpsMode;
      StateId sid;

      if (iter->second.keyframeId.isInitialised()) {
        // reconstruct from close keyframe
        const ViGraph::State &keyframeState = realtimeGraph_.states_.at(iter->second.keyframeId);
        kinematics::Transformation T_WS = keyframeState.pose->estimate() * iter->second.T_Sk_S;
        p_WS_W = T_WS.r();
        q_WS = T_WS.q();
        speedAndBiases.head<3>() = keyframeState.pose->estimate().C() * iter->second.v_Sk;
        speedAndBiases.tail<6>() = keyframeState.speedAndBias->estimate().tail<6>();
        time << iter->second.timestamp.sec << std::setw(9) << std::setfill('0')
             << iter->second.timestamp.nsec;
        nrGps = keyframeState.GpsFactors.size();
        gpsMode = keyframeState.gpsMode;
        sid = iter->first;
      } else {
        // read from state
        const ViGraph::State &state = realtimeGraph_.states_.at(iter->first);
        p_WS_W = state.pose->estimate().r();
        q_WS = state.pose->estimate().q();
        speedAndBiases = state.speedAndBias->estimate();
        time << state.timestamp.sec << std::setw(9) << std::setfill('0')
             << state.timestamp.nsec;
        nrGps = state.GpsFactors.size();
        gpsMode = state.gpsMode;
        sid = iter->first;
      }
      if (rpg) {
        csvFile << std::setprecision(19) << iter->second.timestamp.toSec() << " "
                << p_WS_W[0] << " " << p_WS_W[1] << " " << p_WS_W[2] << " "
                << q_WS.x() << " " << q_WS.y() << " " << q_WS.z() << " " << q_WS.w() << std::endl;
      } else {
        csvFile << time.str() << ", " << std::scientific
                << std::setprecision(18) << p_WS_W[0] << ", " << p_WS_W[1] << ", "
                << p_WS_W[2] << ", " << q_WS.x() << ", " << q_WS.y() << ", "
                << q_WS.z() << ", " << q_WS.w() << ", " << speedAndBiases[0] << ", "
                << speedAndBiases[1] << ", " << speedAndBiases[2] << ", "
                << speedAndBiases[3] << ", " << speedAndBiases[4] << ", "
                << speedAndBiases[5] << ", " << speedAndBiases[6] << ", "
                << speedAndBiases[7] << ", " << speedAndBiases[8] << ", "
                << nrGps << ", " << sid.value() << ", " << gpsMode << ", "
                << iter->second.keyframeId.value() << std::endl;
      }
    }

  }
  // done, close.
  csvFile.close();
  return success;
}

bool ViSlamBackend::writeGlobalCsvTrajectory(const std::string &csvFileName) const
{
  std::fstream csvFile(csvFileName.c_str(), std::ios_base::out);
  bool success =  csvFile.good();
  if(!success) {
    return false;
  }

  //OKVIS_ASSERT_TRUE(Exception, fullGraph_.isSynched(realtimeGraph_), "DA FCK")

  kinematics::Transformation T_GW;

  // write description
  csvFile << "timestamp" << ", " << "p_GA_G_x" << ", " << "p_GA_G_y" << ", "
               << "p_GA_G_z" << std::endl;
  for(auto iter=realtimeGraph_.anyState_.begin(); iter!=realtimeGraph_.anyState_.end(); ++iter) {
    Eigen::Vector3d p_GA_G;
    std::stringstream time;
    kinematics::Transformation T_WS;

    if(iter->second.keyframeId.isInitialised()) {
      // reconstruct from close keyframe
      const ViGraph::State& keyframeState = realtimeGraph_.states_.at(iter->second.keyframeId);
      T_WS = keyframeState.pose->estimate() * iter->second.T_Sk_S;
      time << iter->second.timestamp.sec << std::setw(9) << std::setfill('0')
           << iter->second.timestamp.nsec;
    } else {
      // read from state
      const ViGraph::State& state = realtimeGraph_.states_.at(iter->first);
      T_WS = state.pose->estimate();
      time << state.timestamp.sec << std::setw(9) << std::setfill('0')
           << state.timestamp.nsec;
    }

    T_GW = realtimeGraph_.T_GW();
    p_GA_G =  T_GW.C() * (T_WS.r() + T_WS.C()* realtimeGraph_.gpsParametersVec_.back().r_SA ) + T_GW.r();
    csvFile << time.str() << ", " << std::scientific
        << std::setprecision(18) << p_GA_G[0] << ", " << p_GA_G[1] << ", "
        << p_GA_G[2] <<  std::endl;
  }

  // done, close.
  csvFile.close();
  return success;
}

bool ViSlamBackend::attemptLoopClosure(StateId pose_i, StateId pose_j,
                                       const kinematics::Transformation& T_Si_Sj,
                                       const Eigen::Matrix<double, 6, 6>& information,
                                       bool & skipFullGraphOptimisation,
                                       int numIter, int numThreads, bool verbose)
{
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "Loop closure still running")
  OKVIS_ASSERT_TRUE(Exception, !isLoopClosureAvailable_,
                    "loop closure not finished, cannot merge landmarks")

  // check if poses exist, signal unsuccessful otherwise
  if(realtimeGraph_.states_.count(pose_i) == 0) {
    return false;
  }
  if(realtimeGraph_.states_.count(pose_j) == 0) {
    return false;
  }

  // Step 0: check if this links two components
  size_t matchingComponentIdx = 0;
  bool connectingComponents =  false;
  for(size_t i=0; i< components_.size(); ++i) {
    if(i == currentComponentIdx_) {
      continue;
    }
    if(components_.at(i).poseIds.count(pose_i)) {
      // this means we are connecting components.
      // Remember and remove relative pose links
      matchingComponentIdx = i;
      connectingComponents = true;
      // always merge to smaller index
      if(matchingComponentIdx<currentComponentIdx_) {
        observationLessGraph_.removeRelativePoseConstraint(
              components_.at(currentComponentIdx_).connectedReferenceId,
              components_.at(currentComponentIdx_).referenceId);
      } else {
        observationLessGraph_.removeRelativePoseConstraint(
              components_.at(matchingComponentIdx).connectedReferenceId,
              components_.at(matchingComponentIdx).referenceId);
      }
      break;
    }
  }
  OKVIS_ASSERT_TRUE(Exception, !connectingComponents, "connection components must be disabled")

  // Step 1: try observationLess pose graph optimisation
  // first get all the current observations as two-pose edges
  std::vector<ViGraphEstimator::PoseGraphEdge> poseGraphEdges, poseGraphEdgesAdded;
  realtimeGraph_.obtainPoseGraphMst(poseGraphEdges);
  // clone edges to observationLess!!
  for(const auto & poseGraphEdge : poseGraphEdges) {
    if(!observationLessGraph_.states_.at(poseGraphEdge.referenceId).twoPoseConstLinks.count(
         poseGraphEdge.otherId)) {
      observationLessGraph_.addExternalTwoPoseLink(
            poseGraphEdge.poseGraphError->cloneTwoPoseGraphErrorConst(),
            poseGraphEdge.referenceId, poseGraphEdge.otherId);
      poseGraphEdgesAdded.push_back(poseGraphEdge);
    }
  }

  // and finally the loop closure relative error
#ifndef NDEBUG
  const bool success = observationLessGraph_.addRelativePoseConstraint(
        pose_i, pose_j, T_Si_Sj, information);
  OKVIS_ASSERT_TRUE_DBG(Exception, success, "could not add relative pose constraint")
#else
  observationLessGraph_.addRelativePoseConstraint(
          pose_i, pose_j, T_Si_Sj, information);
#endif

  // freeze / unfreeze
  StateId oldestIdToSetVariable = auxiliaryStates_.at(pose_i).loopId;
  Time oldestT = fullGraph_.states_.at(pose_i).timestamp;
  int ctr = 0;
  StateId oldestId;
  OKVIS_ASSERT_TRUE_DBG(Exception, auxiliaryStates_.count(oldestIdToSetVariable), "veird")
  for(auto iter = auxiliaryStates_.find(oldestIdToSetVariable); ; --iter) {
    if(ctr == numPoseGraphFrames || iter==auxiliaryStates_.begin()) {
      while((oldestT - observationLessGraph_.timestamp(iter->first)).toSec()<minDeltaT) {
        if(iter==auxiliaryStates_.begin()) {
          break;
        }
        --iter;
      }
      observationLessGraph_.unfreezePosesFrom(iter->first);
      if(iter!=auxiliaryStates_.begin()) {
        observationLessGraph_.freezePosesUntil(iter->first);
      }
      observationLessGraph_.unfreezeSpeedAndBiasesFrom(iter->first);
      if(iter!=auxiliaryStates_.begin()) {
        observationLessGraph_.freezeSpeedAndBiasesUntil(iter->first);
      }
      oldestId = iter->first;
      break;
    }
    if(iter==auxiliaryStates_.begin()) {
      observationLessGraph_.unfreezePosesFrom(iter->first);
      observationLessGraph_.unfreezeSpeedAndBiasesFrom(iter->first);
      oldestId = iter->first;
      break;
    }
    ctr++;
  }

  // optimise -- if sensible to do it already now.
  int numVariableStates = 0; // number of varying states
  for(auto iter = observationLessGraph_.states_.find(oldestId);
      iter != observationLessGraph_.states_.end(); ++iter) {
    numVariableStates++;
  }
  int numSteps = 0; // number of steps to distribute error over
  StateId lastLoopId = auxiliaryStates_.at(pose_i).loopId;
  for(auto iter = observationLessGraph_.states_.find(pose_i);
      iter != observationLessGraph_.states_.end(); ++iter) {
    const StateId loopId = auxiliaryStates_.at(iter->first).loopId;
    if(lastLoopId!=loopId) {
      lastLoopId = loopId;
      numSteps++;
    }
  }
  StateId firstId = auxiliaryStates_.at(pose_i).loopId;
  lastLoopId = firstId;
  double distanceTravelled = 0.0;
  kinematics::Transformation T_WS_i = observationLessGraph_.states_.at(pose_i).pose->estimate();
  kinematics::Transformation T_WS_last = observationLessGraph_.states_.at(pose_i).pose->estimate();
  auto iter = observationLessGraph_.states_.find(pose_i);
  std::vector<double> distances;
  ++iter;
  Eigen::Vector3d distanceTravelledVec(0.0,0.0,0.0);
  if(pose_i != lastLoopId) {
    distanceTravelledVec +=
        (observationLessGraph_.states_.at(lastLoopId).pose->estimate().r()-T_WS_i.r());
    distanceTravelled += distanceTravelledVec.norm();

  }
  for(; iter != observationLessGraph_.states_.end(); ++iter) {
    const StateId loopId = auxiliaryStates_.at(iter->first).loopId;
    kinematics::Transformation T_WS_j = iter->second.pose->estimate();
    T_WS_last = T_WS_j;
    if(lastLoopId!=loopId) {
      const Eigen::Vector3d dsVec = (T_WS_j.r()-T_WS_i.r());
      const double ds = dsVec.norm();
      lastLoopId = loopId;
      distances.push_back(ds);
      distanceTravelledVec += dsVec;
      distanceTravelled += ds;
      T_WS_i = T_WS_j;
    }
  }

  skipFullGraphOptimisation = false;
  if(numVariableStates > -1) { /// \todo changed 40->1 to save time: check!

    // too many. we don't optimise, but just re-align the new part to the old part.
    // we distribute the inconsistency along the loop (i.e. what should be variable)
    // according to changing loopIds (i.e. leaving previously closed loops rigid)

    const kinematics::Transformation T_WSi = observationLessGraph_.pose(pose_i);
    const kinematics::Transformation T_WSj_old= observationLessGraph_.pose(pose_j);
    const kinematics::Transformation T_Si_Sj_old = T_WSi.inverse() * T_WSj_old;
    const kinematics::Transformation T_WSj_new = T_WSi * T_Si_Sj;
    kinematics::Transformation T_Wnew_Wold_final = T_WSj_new * T_WSj_old.inverse();

    // compute rotation adjustments
    kinematics::Transformation T_WS_prev = observationLessGraph_.pose(pose_i);
    kinematics::Transformation T_WS = observationLessGraph_.pose(pose_i);
    auto iter = observationLessGraph_.states_.find(pose_i);
    lastLoopId = firstId;
    iter++;
    for( ; iter != observationLessGraph_.states_.end(); ++iter) {
      const kinematics::Transformation T_WSk_old = iter->second.pose->estimate();
      const kinematics::Transformation T_SS = T_WS_prev.inverse()*T_WSk_old;
      T_WS_prev = T_WSk_old;
      const StateId loopId = auxiliaryStates_.at(iter->first).loopId;
      if(lastLoopId!=loopId) {
              T_WS = kinematics::Transformation(
                    Eigen::Vector3d(0,0,0),
                    Eigen::Quaterniond(1,0,0,0).slerp(1.0/double(numSteps), T_Wnew_Wold_final.q()))
                    *  T_WS * T_SS;
        lastLoopId = loopId;
      } else {
        T_WS = T_WS * T_SS;
      }
    }

    const Eigen::Vector3d dr_W = T_WSj_new.r()-T_WS.r();

    // heuristic verification: check relative trajectory errors
    const double relPositionError = dr_W.norm()/(distanceTravelled);
    const double relOrientationError =
        T_WSj_new.q().angularDistance(T_WSj_old.q())/double(numSteps);
    const double relPositionErrorBudget = // [m/m]
            0.0135 + // 1.35% position bias
            0.02*distanceTravelledVec.norm()/distanceTravelled + // 2% scale error
            0.08/sqrt(numSteps); // position noise, 8% stdev per step
    const double relOrientationErrorBudget =
        0.0004 + 0.004/sqrt(numSteps); // bias and noise, in rad/step
    if((relPositionError > relPositionErrorBudget
        || relOrientationError > relOrientationErrorBudget
        || numSteps < 1)
       && !connectingComponents)  {

      // remove all of these relative errors in any case
      /// \todo Don't introduce in the first place...
      observationLessGraph_.removeRelativePoseConstraint(pose_i, pose_j);
      for(const auto & poseGraphEdge : poseGraphEdgesAdded) {
        observationLessGraph_.removeTwoPoseConstLink(
              poseGraphEdge.referenceId, poseGraphEdge.otherId);
      }

      LOG(INFO) << "Skip loop closure (heuristic consistency).";
      LOG(INFO) << "Rel. pos. err. " << relPositionError << " vs budget "
          << relPositionErrorBudget << " m/m, rel. or. err. "
          << relOrientationError << " vs budget "
          << relOrientationErrorBudget << " rad/kf";
      LOG(INFO) << "dist. travelled " << distanceTravelled << " m, no. steps " << numSteps;

      // re-introduce component link, if needed
      if(connectingComponents) {
        Eigen::Matrix<double, 6,6> H = Eigen::Matrix<double, 6,6>::Identity();
        H(0,0) = 100.0; // 10 cm standard deviation
        H(1,1) = 100.0; // 10 cm standard deviation
        H(2,2) = 100.0; // 10 cm standard deviation
        kinematics::Transformation T_SS = kinematics::Transformation::Identity();
        if(matchingComponentIdx<currentComponentIdx_) {
          observationLessGraph_.addRelativePoseConstraint(
                components_.at(currentComponentIdx_).connectedReferenceId,
                components_.at(currentComponentIdx_).referenceId, T_SS, H);
        } else {
          observationLessGraph_.addRelativePoseConstraint(
                components_.at(matchingComponentIdx).connectedReferenceId,
                components_.at(matchingComponentIdx).referenceId, T_SS, H);
        }
      }
      return false;
    }

    // compute full adjustments
    T_WS_prev = observationLessGraph_.pose(pose_i);
    T_WS = observationLessGraph_.pose(pose_i);
    iter = observationLessGraph_.states_.find(pose_i);
    lastLoopId = firstId;
    iter++;
    int ctr = 0;
    double r = 0.0;
    for( ; iter != observationLessGraph_.states_.end(); ++iter) {
      const kinematics::Transformation T_WSk_old = iter->second.pose->estimate();
      const kinematics::Transformation T_SS = T_WS_prev.inverse()*T_WSk_old;
      T_WS_prev = T_WSk_old;
      const StateId loopId = auxiliaryStates_.at(iter->first).loopId;
      if(lastLoopId!=loopId) {
        // we weight distance adjustments by distance travelled, and rotation uniformly.
        r +=  distances.at(size_t(ctr)) / distanceTravelled;
              T_WS = kinematics::Transformation(
                    Eigen::Vector3d(0,0,0),
                    Eigen::Quaterniond(1,0,0,0).slerp(1.0/double(numSteps), T_Wnew_Wold_final.q()))
                    *  T_WS * T_SS;
        lastLoopId = loopId;
        ++ctr;
      } else {
        T_WS = T_WS * T_SS;
      }
      const kinematics::Transformation T_WS_set(T_WS.r() + r*dr_W, T_WS.q());
      const kinematics::Transformation T_Wnew_Wold = T_WS_set * T_WSk_old.inverse();
      SpeedAndBias speedAndBias = iter->second.speedAndBias->estimate();
      const Eigen::Vector3d v_Wold = speedAndBias.head<3>();
      speedAndBias.head<3>() = T_Wnew_Wold.C() * v_Wold;
      realtimeGraph_.setPose(iter->first, T_WS_set);
      fullGraph_.setPose(iter->first, T_WS_set);
      observationLessGraph_.setPose(iter->first, T_WS_set);
      realtimeGraph_.setSpeedAndBias(iter->first, speedAndBias);
      fullGraph_.setSpeedAndBias(iter->first, speedAndBias);
      observationLessGraph_.setSpeedAndBias(iter->first, speedAndBias);
    }

    // optimise
    StateId freezeId;
    for(auto iter = observationLessGraph_.states_.crbegin();
        iter!=observationLessGraph_.states_.crend(); ++iter) {
      if(realtimeGraph_.states_.at(iter->first).pose->fixed()
         && realtimeGraph_.states_.at(iter->first).speedAndBias->fixed()) {
        observationLessGraph_.unfreezePosesFrom(iter->first);
        if(iter->first.value() != 1)
          observationLessGraph_.freezePosesUntil(iter->first);
        observationLessGraph_.unfreezeSpeedAndBiasesFrom(iter->first);
        if(iter->first.value() != 1)
          observationLessGraph_.freezeSpeedAndBiasesUntil(iter->first);
        freezeId = iter->first;
        break;
      }
    }

    // signal correct fixation
    if(pose_i.value() < freezeId.value()) {
      observationLessGraph_.unfreezePosesFrom(pose_i);
      if(pose_i.value() != 1)
        observationLessGraph_.freezePosesUntil(pose_i);
      observationLessGraph_.unfreezeSpeedAndBiasesFrom(pose_i);
      if(pose_i.value() != 1)
        observationLessGraph_.freezeSpeedAndBiasesUntil(pose_i);
    }

    // remove all of these relative errors in any case
    /// \todo Don't introduce in the first place...
    observationLessGraph_.removeRelativePoseConstraint(pose_i, pose_j);
    for(const auto & poseGraphEdge : poseGraphEdgesAdded) {
      observationLessGraph_.removeTwoPoseConstLink(
            poseGraphEdge.referenceId, poseGraphEdge.otherId);
    }

    /// update landmarks
    for(auto iter = realtimeGraph_.landmarks_.begin();
        iter != realtimeGraph_.landmarks_.end(); ++iter) {
      // TODO: check if this is always right!
      Eigen::Vector4d hPointNew = T_Wnew_Wold_final * iter->second.hPoint->estimate();
      realtimeGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
      fullGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
    }

    LOG(INFO) << "Long loop closure";
  } else {
    // optimise observation-less
    const kinematics::Transformation T_Wold_S= observationLessGraph_.pose(pose_j);
    observationLessGraph_.options().function_tolerance = 0.01;
    observationLessGraph_.optimise(numIter, numThreads, verbose);

    const kinematics::Transformation T_Wnew_S = observationLessGraph_.pose(pose_j);
    const kinematics::Transformation T_Wnew_Wold = T_Wnew_S * T_Wold_S.inverse();

    // remove all of these relative errors in any case
    observationLessGraph_.removeRelativePoseConstraint(pose_i, pose_j);
    for(const auto & poseGraphEdge : poseGraphEdgesAdded) {
      observationLessGraph_.removeTwoPoseConstLink(
            poseGraphEdge.referenceId, poseGraphEdge.otherId);
    }

    // check consistency
    auto summary = observationLessGraph_.summary();
    int k = summary.num_residuals_reduced - summary.num_effective_parameters_reduced; // Chi2 DoF
    if(k<=0) {
      k=1;
    }
    const double normalised_test_sample = (2*(summary.final_cost-summary.fixed_cost) - k)
        /sqrt(2.0*double(k));
    if(normalised_test_sample > 4.0) {
      // consistency not passed -- revert the result
      for(auto riter = observationLessGraph_.states_.crbegin();
          riter != observationLessGraph_.states_.crend(); ++riter) {
        if(riter->second.pose->fixed() && riter->second.speedAndBias->fixed()) {
          break;
        }
        observationLessGraph_.setPose(riter->first, fullGraph_.pose(riter->first));
        observationLessGraph_.setSpeedAndBias(riter->first, fullGraph_.speedAndBias(riter->first));
        for(size_t i = 0; i<riter->second.extrinsics.size(); ++i) {
          if(!riter->second.extrinsics.at(i)->fixed()) {
            observationLessGraph_.setExtrinsics(riter->first, uchar(i),
                                         fullGraph_.extrinsics(riter->first, uchar(i)));
          }
        }
      }
      // re-introduce component link, if needed
      if(connectingComponents) {
        Eigen::Matrix<double, 6,6> H = Eigen::Matrix<double, 6,6>::Identity();
        H(0,0) = 100.0; // 10 cm standard deviation
        H(1,1) = 100.0; // 10 cm standard deviation
        H(2,2) = 100.0; // 10 cm standard deviation
        kinematics::Transformation T_SS = kinematics::Transformation::Identity();
        if(matchingComponentIdx<currentComponentIdx_) {
          observationLessGraph_.addRelativePoseConstraint(
                components_.at(currentComponentIdx_).connectedReferenceId,
                components_.at(currentComponentIdx_).referenceId, T_SS, H);
        } else {
          observationLessGraph_.addRelativePoseConstraint(
                components_.at(matchingComponentIdx).connectedReferenceId,
                components_.at(matchingComponentIdx).referenceId, T_SS, H);
        }

      }
      LOG(INFO) << "skip loop closure after consistency check " << normalised_test_sample
                << " mean cost per residual " << 2*summary.final_cost/k;
      return false;
    }

    // consistency passed -- copy the result over now
    for(auto riter = observationLessGraph_.states_.crbegin();
        riter != observationLessGraph_.states_.crend(); ++riter) {
      if(riter->second.pose->fixed() && riter->second.speedAndBias->fixed()
         && realtimeGraph_.states_.at(riter->first).pose->fixed()
         && realtimeGraph_.states_.at(riter->first).speedAndBias->fixed()
         && fullGraph_.states_.at(riter->first).pose->fixed()
         && fullGraph_.states_.at(riter->first).speedAndBias->fixed()
         ) {
        break;
      }
      realtimeGraph_.setPose(riter->first, riter->second.pose->estimate());
      fullGraph_.setPose(riter->first, riter->second.pose->estimate());
      realtimeGraph_.setSpeedAndBias(riter->first, riter->second.speedAndBias->estimate());
      fullGraph_.setSpeedAndBias(riter->first, riter->second.speedAndBias->estimate());
      for(size_t i = 0; i<riter->second.extrinsics.size(); ++i) {
        if(!riter->second.extrinsics.at(i)->fixed()) {
          realtimeGraph_.setExtrinsics(riter->first, uchar(i),
                                       riter->second.extrinsics.at(i)->estimate());
          fullGraph_.setExtrinsics(riter->first, uchar(i),
                                   riter->second.extrinsics.at(i)->estimate());
        }
      }
    }

    /// update landmarks
    for(auto iter = realtimeGraph_.landmarks_.begin(); iter != realtimeGraph_.landmarks_.end();
        ++iter) {
      Eigen::Vector4d hPointNew = T_Wnew_Wold * iter->second.hPoint->estimate();
      realtimeGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
      fullGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
    }
  }

  // remember this frame closed a loop
  auxiliaryStates_.at(pose_j).closedLoop = true;

  // merge components, if necesary
  if(connectingComponents) {
    std::set<StateId> currentComponentIds = components_.at(currentComponentIdx_).poseIds;
    if(matchingComponentIdx<currentComponentIdx_) {
      realtimeGraph_.removeRelativePoseConstraint(
            components_.at(currentComponentIdx_).connectedReferenceId,
            components_.at(currentComponentIdx_).referenceId);
      fullGraph_.removeRelativePoseConstraint(
            components_.at(currentComponentIdx_).connectedReferenceId,
            components_.at(currentComponentIdx_).referenceId);
      components_.at(matchingComponentIdx).poseIds.insert(
            currentComponentIds.begin(), currentComponentIds.end());
      components_.erase(components_.begin()+int(currentComponentIdx_));
      currentComponentIdx_ = matchingComponentIdx;
    } else {
      realtimeGraph_.removeRelativePoseConstraint(
            components_.at(matchingComponentIdx).connectedReferenceId,
            components_.at(matchingComponentIdx).referenceId);
      fullGraph_.removeRelativePoseConstraint(
            components_.at(matchingComponentIdx).connectedReferenceId,
            components_.at(matchingComponentIdx).referenceId);
      components_.at(currentComponentIdx_).poseIds.insert(
            currentComponentIds.begin(), currentComponentIds.end());
      components_.erase(components_.begin()+int(matchingComponentIdx));
    }
  }

  fullGraphRelativePoseConstraints_.push_back(RelPoseInfo{T_Si_Sj, information, pose_i, pose_j});

  //OKVIS_ASSERT_TRUE(Exception, fullGraph_.isSynched(realtimeGraph_), "sdfjhasieu")
  return true;
}

bool ViSlamBackend::attemptGpsAlignment(StateId pose_i , StateId pose_j){
    OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "Loop closure still running")
    OKVIS_ASSERT_TRUE(Exception, !isLoopClosureAvailable_, "loop closure not finished, cannot merge landmarks")
    //OKVIS_ASSERT_TRUE(Exception, realtimeGraph_.isObservationlessSynched(observationLessGraph_), "bad");

    // check if poses exist, signal unsuccessful otherwise
    if(realtimeGraph_.states_.count(pose_i) == 0) {
      return false;
    }
    if(realtimeGraph_.states_.count(pose_j) == 0) {
      return false;
    }

    // Obtain different T_GW estimates and compute relative transformations
    okvis::kinematics::Transformation T_GW_old = realtimeGraph_.T_GW(pose_i);
    okvis::kinematics::Transformation T_GW_new = realtimeGraph_.T_GW(pose_j);
    std::cout << "[DEBUG Info GPS Alignment] Trying to close loop with \n"
              << "T_GW_old:\n" << T_GW_old.T3x4() << "\n"
              << "T_GW_new:\n" << T_GW_new.T3x4() << std::endl;
    std::cout << "[DEBUG Info GPS Alignment] Loop is closed between sid "
              << pose_i.value() << " and " << pose_j.value() << " while most recent state is " << realtimeGraph_.states_.rbegin()->first.value() << std::endl;


    okvis::kinematics::Transformation T_WSj = observationLessGraph_.pose(pose_j);
    okvis::kinematics::Transformation T_WSi = observationLessGraph_.pose(pose_i);

    okvis::kinematics::Transformation T_GSj_old = T_GW_old * T_WSj;
    okvis::kinematics::Transformation T_GSj_new = T_GW_new * T_WSj;
    okvis::kinematics::Transformation T_Sj_old_Sj_new = T_GSj_old.inverse() * T_GSj_new; // new

    // okvis::kinematics::Transformation T_SiSj_new = T_WSi.inverse() * T_GW_old.inverse() * T_GSj_new;
    okvis::kinematics::Transformation T_SiSj_old = T_WSi.inverse() * T_WSj;
    okvis::kinematics::Transformation T_SiSj_new = T_SiSj_old * T_Sj_old_Sj_new;


    const kinematics::Transformation T_WSj_new = T_WSi * T_SiSj_new;
    //kinematics::Transformation T_Wnew_Wold_final = T_WSj_new * T_WSj.inverse();
    kinematics::Transformation T_Wnew_Wold_final = T_GW_new.inverse() * T_GW_old;
    T_Wnew_Wold_final = T_Wnew_Wold_final.inverse();



    // freeze / unfreeze
    int ctr = 0;
    StateId oldestId=pose_i;

    observationLessGraph_.unfreezePosesFrom(pose_i);
    observationLessGraph_.freezePosesUntil(pose_i);

    observationLessGraph_.unfreezeSpeedAndBiasesFrom(pose_i);
    observationLessGraph_.freezeSpeedAndBiasesUntil(pose_i);

    // optimise -- if sensible to do it already now.
    int numVariableStates = 0; // number of varying states
    for(auto iter = observationLessGraph_.states_.find(oldestId);
        iter != observationLessGraph_.states_.end(); ++iter) {
      numVariableStates++;
    }

    StateId firstId = auxiliaryStates_.at(pose_i).loopId;
    StateId lastId;
    StateId id;
    lastId = firstId;

    double distanceTravelled = 0.0;
    double fullDistanceTravelled = 0.0;
    kinematics::Transformation T_WS_i = observationLessGraph_.states_.at(pose_i).pose->estimate();
    kinematics::Transformation T_WS_last = observationLessGraph_.states_.at(pose_i).pose->estimate();
    auto iter = observationLessGraph_.states_.find(pose_i);
    std::vector<double> distances;
    ++iter;
    for(; iter != observationLessGraph_.states_.end(); ++iter) {
      kinematics::Transformation T_WS_j = iter->second.pose->estimate();
      const double full_ds = (T_WS_last.r()-T_WS_i.r()).norm();
      T_WS_last = T_WS_j;
      fullDistanceTravelled += full_ds;
      id = iter->first;
      double ds =
          (observationLessGraph_.pose(id).r()-observationLessGraph_.pose(lastId).r()).norm();
      lastId = id;
      ds = std::max(ds, (T_WS_j.r()-T_WS_i.r()).norm());
      distances.push_back(ds);
      distanceTravelled += ds;
      T_WS_i = T_WS_j;
    }

  // too many. we don't optimise, but just re-align the new part to the old part.
  // we distribute the inconsistency along the loop (i.e. what should be variable)
  // according to changing loopIds (i.e. leaving previously closed loops rigid)
  //double cost = 0.0;
  //observationLessGraph_.problem_->Evaluate(::ceres::Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
  //std::cout << "cost0 = " << cost << std::endl;

  // compute rotation adjustments
  kinematics::Transformation T_WS_prev = observationLessGraph_.pose(pose_i);
  kinematics::Transformation T_WS = observationLessGraph_.pose(pose_i);
  iter = observationLessGraph_.states_.find(pose_i);
  lastId = firstId;
  iter++;
  for( ; iter != observationLessGraph_.states_.end(); ++iter) {
    const kinematics::Transformation T_WSk_old = iter->second.pose->estimate();
    const kinematics::Transformation T_SS = T_WS_prev.inverse()*T_WSk_old;
    T_WS_prev = T_WSk_old;
    id = iter->first;
    T_WS = kinematics::Transformation(
          Eigen::Vector3d(0,0,0),
          Eigen::Quaterniond(1,0,0,0).slerp(1.0/double(numVariableStates), T_Wnew_Wold_final.q()))
          *  T_WS * T_SS;
    lastId = id;
  }

  //const kinematics::Transformation T_WW = T_WSj_new * T_WS.inverse();
  const Eigen::Vector3d dr_W = T_WSj_new.r()-T_WS.r();

  // compute full adjustments
  T_WS_prev = observationLessGraph_.pose(pose_i);
  T_WS = observationLessGraph_.pose(pose_i);
  iter = observationLessGraph_.states_.find(pose_i);
  lastId = firstId;
  iter++;
  ctr = 0;
  double r = 0.0;
  for( ; iter != observationLessGraph_.states_.end(); ++iter) {
    const kinematics::Transformation T_WSk_old = iter->second.pose->estimate();
    const kinematics::Transformation T_SS = T_WS_prev.inverse()*T_WSk_old;
    T_WS_prev = T_WSk_old;
    id = iter->first;
    // we weight distance adjustments by distance travelled, and rotation uniformly.
    r +=  distances.at(ctr) / distanceTravelled;
          T_WS = kinematics::Transformation(
                Eigen::Vector3d(0,0,0),
                Eigen::Quaterniond(1,0,0,0).slerp(1.0/double(numVariableStates), T_Wnew_Wold_final.q()))
                *  T_WS * T_SS;
    lastId = id;
    ++ctr;
    const kinematics::Transformation T_WS_set(T_WS.r() + r*dr_W, T_WS.q());
    const kinematics::Transformation T_Wnew_Wold = T_WS_set * T_WSk_old.inverse();
    SpeedAndBias speedAndBias = iter->second.speedAndBias->estimate();
    speedAndBias.head<3>() = (T_Wnew_Wold.C() * speedAndBias.head<3>()).eval();
    std::cout << "[DEBUG Info GPS Alignment] realtimeGraph_ setting pose from \n"
              << realtimeGraph_.pose(iter->first).T3x4() << "\n"
              << "to:\n" << T_WS_set.T3x4() << std::endl;
    realtimeGraph_.setPose(iter->first, T_WS_set);
    std::cout << "[DEBUG Info GPS Alignment] fullGraph_ setting pose from \n"
              << fullGraph_.pose(iter->first).T3x4() << "\n"
              << "to:\n" << T_WS_set.T3x4() << std::endl;
    fullGraph_.setPose(iter->first, T_WS_set);
    std::cout << "[DEBUG Info GPS Alignment] observationLessGraph_ setting pose from \n"
              << observationLessGraph_.pose(iter->first).T3x4() << "\n"
              << "to:\n" << T_WS_set.T3x4() << std::endl;
    observationLessGraph_.setPose(iter->first, T_WS_set);
    realtimeGraph_.setSpeedAndBias(iter->first, speedAndBias);
    fullGraph_.setSpeedAndBias(iter->first, speedAndBias);
    observationLessGraph_.setSpeedAndBias(iter->first, speedAndBias);
  }
  //std::cout << "numSteps = " << numSteps << " T_WS=" << T_WS.T() << std::endl;
  //std::cout << " T_err_j=" << (realtimeGraph_.pose(pose_j)*T_WSj_new.inverse()).T() << std::endl;

  // optimise
  StateId freezeId;
  for(auto iter = observationLessGraph_.states_.crbegin();
      iter!=observationLessGraph_.states_.crend(); ++iter) {
    if(realtimeGraph_.states_.at(iter->first).pose->fixed()
       && realtimeGraph_.states_.at(iter->first).speedAndBias->fixed()) {
      observationLessGraph_.unfreezePosesFrom(iter->first);
      if(iter->first.value() != 1)
        observationLessGraph_.freezePosesUntil(iter->first);
      observationLessGraph_.unfreezeSpeedAndBiasesFrom(iter->first);
      if(iter->first.value() != 1)
        observationLessGraph_.freezeSpeedAndBiasesUntil(iter->first);
      freezeId = iter->first;
      break;
    }
  }

  // signal correct fixation
  if(pose_i.value() < freezeId.value()) {
    observationLessGraph_.unfreezePosesFrom(pose_i);
    if(pose_i.value() != 1)
      observationLessGraph_.freezePosesUntil(pose_i);
    observationLessGraph_.unfreezeSpeedAndBiasesFrom(pose_i);
    if(pose_i.value() != 1)
      observationLessGraph_.freezeSpeedAndBiasesUntil(pose_i);
  }

  /// update landmarks
  for(auto iter = realtimeGraph_.landmarks_.begin(); iter != realtimeGraph_.landmarks_.end(); ++iter) {
    Eigen::Vector4d hPointNew = T_Wnew_Wold_final * iter->second.hPoint->estimate();
    realtimeGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
    fullGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
  }

  std::cout << "GPS loop closure" << std::endl;
  std::cout << "[DEBUG INFO GPS Alignment] Last Pose of Alignment: " << realtimeGraph_.pose(pose_j).T3x4() << std::endl;
  std::cout << "[DEBUG INFO GPS Alignment] Most recent pose: " << realtimeGraph_.pose(realtimeGraph_.states_.rbegin()->first).T3x4() << std::endl;
    return true;
}

bool ViSlamBackend::attemptFullGpsAlignment(StateId pose_i , StateId pose_j, const okvis::kinematics::Transformation& T_GW_new){
    OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "Loop closure still running")
    OKVIS_ASSERT_TRUE(Exception, !isLoopClosureAvailable_, "loop closure not finished, cannot merge landmarks")
    //OKVIS_ASSERT_TRUE(Exception, realtimeGraph_.isObservationlessSynched(observationLessGraph_), "bad");

    // check if poses exist, signal unsuccessful otherwise
    if(realtimeGraph_.states_.count(pose_i) == 0) {
      return false;
    }
    if(realtimeGraph_.states_.count(pose_j) == 0) {
      return false;
    }

    // Obtain different T_GW estimates and compute relative transformations
    okvis::kinematics::Transformation T_GW_old = realtimeGraph_.T_GW(pose_i);
    //okvis::kinematics::Transformation T_GW_new = realtimeGraph_.T_GW(pose_j);
    std::cout << "[DEBUG Info GPS Alignment] Full GPS Alignment! Trying to close loop with \n"
              << "T_GW_old:\n" << T_GW_old.T3x4() << "\n"
              << "T_GW_new:\n" << T_GW_new.T3x4() << std::endl;
    std::cout << "[DEBUG Info GPS Alignment] Loop is closed between sid "
              << pose_i.value() << " and " << pose_j.value() << " while most recent state is " << realtimeGraph_.states_.rbegin()->first.value() << std::endl;


    okvis::kinematics::Transformation T_WSj = observationLessGraph_.pose(pose_j);
    okvis::kinematics::Transformation T_WSi = observationLessGraph_.pose(pose_i);

    okvis::kinematics::Transformation T_GSj_old = T_GW_old * T_WSj;
    okvis::kinematics::Transformation T_GSj_new = T_GW_new * T_WSj;
    okvis::kinematics::Transformation T_Sj_old_Sj_new = T_GSj_old.inverse() * T_GSj_new; // new

    // okvis::kinematics::Transformation T_SiSj_new = T_WSi.inverse() * T_GW_old.inverse() * T_GSj_new;
    okvis::kinematics::Transformation T_SiSj_old = T_WSi.inverse() * T_WSj;
    okvis::kinematics::Transformation T_SiSj_new = T_SiSj_old * T_Sj_old_Sj_new;


    const kinematics::Transformation T_WSj_new = T_WSi * T_SiSj_new;
    //kinematics::Transformation T_Wnew_Wold_final = T_WSj_new * T_WSj.inverse();
    kinematics::Transformation T_Wnew_Wold_final = T_GW_new.inverse() * T_GW_old;
    T_Wnew_Wold_final = T_Wnew_Wold_final.inverse();



    // freeze / unfreeze
    int ctr = 0;
    StateId oldestId=pose_i;

    observationLessGraph_.unfreezePosesFrom(pose_i);
    observationLessGraph_.freezePosesUntil(pose_i);

    observationLessGraph_.unfreezeSpeedAndBiasesFrom(pose_i);
    observationLessGraph_.freezeSpeedAndBiasesUntil(pose_i);

    // optimise -- if sensible to do it already now.
    int numVariableStates = 0; // number of varying states
    for(auto iter = observationLessGraph_.states_.find(oldestId);
        iter != observationLessGraph_.states_.end(); ++iter) {
      numVariableStates++;
    }

    StateId firstId = auxiliaryStates_.at(pose_i).loopId;
    std::cout << "firstId " << pose_i.value() << " and loopId " << firstId.value() << std::endl;
    StateId lastId;
    StateId id;
    lastId = firstId;

    double distanceTravelled = 0.0;
    double fullDistanceTravelled = 0.0;
    kinematics::Transformation T_WS_i = observationLessGraph_.states_.at(pose_i).pose->estimate();
    kinematics::Transformation T_WS_last = observationLessGraph_.states_.at(pose_i).pose->estimate();
    auto iter = observationLessGraph_.states_.find(pose_i);
    std::vector<double> distances;
    ++iter;
    for(; iter != observationLessGraph_.states_.end(); ++iter) {
      kinematics::Transformation T_WS_j = iter->second.pose->estimate();
      const double full_ds = (T_WS_last.r()-T_WS_j.r()).norm();
      T_WS_last = T_WS_j;
      fullDistanceTravelled += full_ds;
      id = iter->first;
      double ds = (T_WS_j.r()-T_WS_i.r()).norm();
      distances.push_back(ds);
      distanceTravelled += ds;
      T_WS_i = T_WS_j;
    }

  // compute rotation adjustments
  kinematics::Transformation T_WS_prev = observationLessGraph_.pose(pose_i);
  kinematics::Transformation T_WS = observationLessGraph_.pose(pose_i);
  iter = observationLessGraph_.states_.find(pose_i);
  lastId = firstId;
  iter++;
  for( ; iter != observationLessGraph_.states_.end(); ++iter) {
    const kinematics::Transformation T_WSk_old = iter->second.pose->estimate();
    const kinematics::Transformation T_SS = T_WS_prev.inverse()*T_WSk_old;
    T_WS_prev = T_WSk_old;
    id = iter->first;
    T_WS = kinematics::Transformation(
          Eigen::Vector3d(0,0,0),
          Eigen::Quaterniond(1,0,0,0).slerp(1.0/double(numVariableStates), T_Wnew_Wold_final.q()))
          *  T_WS * T_SS;
    lastId = id;
  }

  //const kinematics::Transformation T_WW = T_WSj_new * T_WS.inverse();
  const Eigen::Vector3d dr_W = T_WSj_new.r()-T_WS.r();

  // compute full adjustments
  T_WS_prev = observationLessGraph_.pose(pose_i);
  T_WS = observationLessGraph_.pose(pose_i);
  iter = observationLessGraph_.states_.find(pose_i);
  lastId = firstId;
  iter++;
  ctr = 0;
  double r = 0.0;
  for( ; iter != observationLessGraph_.states_.end(); ++iter) {
    const kinematics::Transformation T_WSk_old = iter->second.pose->estimate();
    const kinematics::Transformation T_SS = T_WS_prev.inverse()*T_WSk_old;
    T_WS_prev = T_WSk_old;
    id = iter->first;
    // we weight distance adjustments by distance travelled, and rotation uniformly.
    r +=  distances.at(ctr) / distanceTravelled;
    std::cout << "r = " << r << std::endl;
          T_WS = kinematics::Transformation(
                Eigen::Vector3d(0,0,0),
                Eigen::Quaterniond(1,0,0,0).slerp(1.0/double(numVariableStates), T_Wnew_Wold_final.q()))
                *  T_WS * T_SS;
    lastId = id;
    ++ctr;
    const kinematics::Transformation T_WS_set(T_WS.r() + r*dr_W, T_WS.q());
    const kinematics::Transformation T_Wnew_Wold = T_WS_set * T_WSk_old.inverse();
    SpeedAndBias speedAndBias = iter->second.speedAndBias->estimate();
    speedAndBias.head<3>() = (T_Wnew_Wold.C() * speedAndBias.head<3>()).eval();
    realtimeGraph_.setPose(iter->first, T_WS_set);
    fullGraph_.setPose(iter->first, T_WS_set);
    observationLessGraph_.setPose(iter->first, T_WS_set);
    realtimeGraph_.setSpeedAndBias(iter->first, speedAndBias);
    fullGraph_.setSpeedAndBias(iter->first, speedAndBias);
    observationLessGraph_.setSpeedAndBias(iter->first, speedAndBias);
  }

  /// update landmarks
  for(auto iter = realtimeGraph_.landmarks_.begin(); iter != realtimeGraph_.landmarks_.end(); ++iter) {
    Eigen::Vector4d hPointNew = T_Wnew_Wold_final * iter->second.hPoint->estimate();
    realtimeGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
    fullGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
  }

  std::cout << "GPS loop closure" << std::endl;
    return true;
}

bool ViSlamBackend::attemptPosGpsAlignment(StateId pose_i , StateId pose_j, const Eigen::Vector3d& posAlignVec){
    OKVIS_ASSERT_TRUE(Exception, !isLoopClosing_, "Loop closure still running")
    OKVIS_ASSERT_TRUE(Exception, !isLoopClosureAvailable_, "loop closure not finished, cannot merge landmarks")
    //OKVIS_ASSERT_TRUE(Exception, realtimeGraph_.isObservationlessSynched(observationLessGraph_), "bad");

    // check if poses exist, signal unsuccessful otherwise
    if(realtimeGraph_.states_.count(pose_i) == 0) {
      return false;
    }
    if(realtimeGraph_.states_.count(pose_j) == 0) {
      return false;
    }

    std::cout << "[DEBUG INFO GPS Alignment] Trying to align positions between " << pose_i.value() << " and " << pose_j.value()
              << " with vector\n" << posAlignVec << std::endl;
    Eigen::Vector3d r_Wold_Wnew = posAlignVec;

    // freeze / unfreeze
    StateId oldestId=pose_i;

    observationLessGraph_.unfreezePosesFrom(pose_i);
    observationLessGraph_.freezePosesUntil(pose_i);

    observationLessGraph_.unfreezeSpeedAndBiasesFrom(pose_i);
    observationLessGraph_.freezeSpeedAndBiasesUntil(pose_i);

    // optimise -- if sensible to do it already now.
    int numVariableStates = 0; // number of varying states
    for(auto iter = observationLessGraph_.states_.find(oldestId);
        iter != observationLessGraph_.states_.end(); ++iter) {
      numVariableStates++;
      if(iter->first == pose_j){
          std::cout << " breaking variablestates loop  " << std::endl;
          break;
        }
    }

    StateId lastId;
    StateId id;
    lastId = oldestId;

    double fullDistanceTravelled = 0.0;
    std::vector<double> distances; // save distance travelled per step
    auto iter = observationLessGraph_.states_.find(pose_i);
    ++iter;
    for(; iter != observationLessGraph_.states_.end(); ++iter) {
      id = iter->first;
      if(id.value() > pose_j.value())
        break;
      double ds = (observationLessGraph_.pose(id).r()-observationLessGraph_.pose(lastId).r()).norm();
      distances.push_back(ds);
      lastId = id;
      fullDistanceTravelled += ds;
    }

    // Now apply position adjustments based on distance travelled
    size_t counter = 0; // counter to iterate distances vector
    iter = observationLessGraph_.states_.find(pose_i);
    lastId = oldestId;
    ++iter;
    double distanceTravelled = 0.0;
    okvis::kinematics::Transformation T_WS;

    for(; iter != observationLessGraph_.states_.end(); ++iter) {
      id = iter->first;
      //std::cout << "Adjusting state " << id.value() << std::endl;
      if(id.value() <= pose_j.value()){

          distanceTravelled += distances.at(counter);

          // position adjustment
          std::cout << "Adjustment is done with a factor " << distanceTravelled/fullDistanceTravelled << std::endl;
          Eigen::Vector3d dr = distanceTravelled/fullDistanceTravelled * r_Wold_Wnew;
          // now apply position adjustment
          T_WS = observationLessGraph_.pose(id);
          okvis::kinematics::Transformation T_WS_set(T_WS.r()+dr,T_WS.q());
          realtimeGraph_.setPose(iter->first, T_WS_set);
          fullGraph_.setPose(iter->first, T_WS_set);
          observationLessGraph_.setPose(iter->first, T_WS_set);
          counter+=1;
        }
      else{
          // now apply position adjustment
          //std::cout << "Adjustment is done with a rigid factor of 1 " << std::endl;
          T_WS = observationLessGraph_.pose(id);
          okvis::kinematics::Transformation T_WS_set(T_WS.r() + r_Wold_Wnew,T_WS.q());
          realtimeGraph_.setPose(iter->first, T_WS_set);
          fullGraph_.setPose(iter->first, T_WS_set);
          observationLessGraph_.setPose(iter->first, T_WS_set);
        }
    }

  /// update landmarks
  for(auto iter = realtimeGraph_.landmarks_.begin(); iter != realtimeGraph_.landmarks_.end(); ++iter) {
    Eigen::Vector4d r_Wold_Wnew_hom(r_Wold_Wnew(0) , r_Wold_Wnew(1) , r_Wold_Wnew(2) , 0.0);
    Eigen::Vector4d hPointNew = r_Wold_Wnew_hom + iter->second.hPoint->estimate();
    realtimeGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
    fullGraph_.setLandmark(iter->first, hPointNew, iter->second.hPoint->initialized());
  }

  return true;
}

bool ViSlamBackend::addSubmapAlignmentConstraints(const se::OccupancyMap<se::Res::Multi>* submap_ptr,
                                                  const uint64_t &frame_A_id, const uint64_t frame_B_id,
                                                  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& pointCloud) {
  return realtimeGraph_.addSubmapAlignmentConstraints(submap_ptr, frame_A_id, frame_B_id, pointCloud);
}

StateId ViSlamBackend::currentKeyframeStateId(bool considerLoopClosureFrames) const
{
  StateId currentFrame = currentStateId();
  return mostOverlappedStateId(currentFrame, considerLoopClosureFrames);
}

StateId ViSlamBackend::mostOverlappedStateId(StateId frame, bool considerLoopClosureFrames) const
{
  std::set<StateId> allFrames;
  allFrames.insert(keyFrames_.begin(), keyFrames_.end());
  allFrames.insert(imuFrames_.begin(), imuFrames_.end());
  allFrames.insert(loopClosureFrames_.begin(), loopClosureFrames_.end());
  if(!considerLoopClosureFrames) {
    for(const auto & id : currentLoopClosureFrames_)
      allFrames.erase(id);
  }
  StateId returnId;
  double overlap = 0.0;
  for(auto id : allFrames) {
    if(id==frame) {
      continue;
    }
    if(!realtimeGraph_.states_.at(id).isKeyframe) {
      continue;
    }
    double thisOverlap = overlapFraction(multiFrames_.at(id), multiFrames_.at(frame));
    if(thisOverlap >= overlap) {
      returnId = id;
      overlap = thisOverlap;
    }
  }
  if(returnId==frame) {
    OKVIS_THROW(Exception, "bug")
  }
  return returnId;
}

StateId ViSlamBackend::currentLoopclosureStateId() const
{
  StateId currentFrame = currentStateId();
  StateId returnId;
  double overlap = 0.0;
  for(auto id : loopClosureFrames_) {
    if(id==currentFrame) {
      continue;
    }
    if(!realtimeGraph_.states_.at(id).isKeyframe) {
      continue;
    }
    double thisOverlap = overlapFraction(multiFrames_.at(id), multiFrames_.at(currentFrame));
    if(thisOverlap >= overlap) {
      returnId = id;
      overlap = thisOverlap;
    }
  }
  if(returnId==currentFrame) {
    OKVIS_THROW(Exception, "bug")
  }
  if(overlap>0.5) {
    return returnId;
  }
  return StateId();
}

int ViSlamBackend::prunePlaceRecognitionFrames() {
  realtimeGraph_.computeCovisibilities();
  std::set<StateId> allFrames;
  allFrames.insert(keyFrames_.begin(), keyFrames_.end());
  allFrames.insert(loopClosureFrames_.begin(), loopClosureFrames_.end());
  int ctr=0;
  for(auto id0 : allFrames) {
    for(auto id1 : allFrames) {
      if(id0.value()<=id1.value()) {
        continue;
      }
      if(!auxiliaryStates_.at(id0).isPlaceRecognitionFrame) {
        continue;
      }
      if(!auxiliaryStates_.at(id1).isPlaceRecognitionFrame) {
        continue;
      }
      if(realtimeGraph_.covisibilities(id0, id1) < 10) {
        continue;
      }
      const double overlap = overlapFraction(multiFrames_.at(id0), multiFrames_.at(id1));
      if(overlap > 0.6) {
        ctr++;
        // prune the newer frame
        auxiliaryStates_.at(id0).isPlaceRecognitionFrame = false;
      }
    }
  }
  return ctr;
}

void ViSlamBackend::clear()
{
  // clear underlying graphs
  realtimeGraph_.clear();
  fullGraph_.clear();
  observationLessGraph_.clear();

  multiFrames_.clear();

  auxiliaryStates_.clear(); // Store information about states.
  currentComponentIdx_ = 0; // The index of the current component.

  loopClosureFrames_.clear(); // All the current loop closure frames.

  imuFrames_.clear(); // All the current IMU frames.
  keyFrames_.clear(); // All the current keyframes.

  needsFullGraphOptimisation_ = false;
  isLoopClosing_ = false;
  isLoopClosureAvailable_ = false;
  components_.resize(1);

  addStatesBacklog_.clear(); // Backlog of states to add to fullGraph_.
  eliminateStates_.clear(); // States eliminated in realtimeGraph_.
  touchedStates_.clear(); // States modified in realtimeGraph_.
  touchedLandmarks_.clear(); // Landmarks modified in realtimeGraph_.

  fullGraphRelativePoseConstraints_.clear(); // Relative pose constraints.

  lastFreeze_ = StateId(); // Store up to where the realtimeGraph_ states were fixed.
}

double ViSlamBackend::overlapFraction(const MultiFramePtr frameA,
                                      const MultiFramePtr frameB) const {

  OKVIS_ASSERT_TRUE(Exception, frameA->numFrames() == frameB->numFrames(),
                    "must be same number of frames")
  const size_t numFrames = frameA->numFrames();
  const MultiFramePtr frames[2] = {frameA, frameB};

  std::set<LandmarkId> landmarks[2];
  std::vector<cv::Mat> detectionsImg[2];
  detectionsImg[0].resize(numFrames);
  detectionsImg[1].resize(numFrames);
  std::vector<cv::Mat> matchesImg[2];
  matchesImg[0].resize(numFrames);
  matchesImg[1].resize(numFrames);

  // paint detection images and remember matched points
  for(size_t f=0; f<2; ++f) {
    for (size_t im = 0; im < frames[f]->numFrames(); ++im) {
      const int rows = frames[f]->image(im).rows/10;
      const int cols = frames[f]->image(im).cols/10;
      const double radius = double(std::min(rows,cols))*kptradius_;
      detectionsImg[f].at(im) = cv::Mat::zeros(rows, cols, CV_8UC1);
      matchesImg[f].at(im) = cv::Mat::zeros(rows, cols, CV_8UC1);
      const size_t num = frames[f]->numKeypoints(im);
      cv::KeyPoint keypoint;
      for (size_t k = 0; k < num; ++k) {
        frames[f]->getCvKeypoint(im, k, keypoint);
        cv::circle(detectionsImg[f].at(im), keypoint.pt*0.1, int(radius), cv::Scalar(255),
                   cv::FILLED);
        uint64_t lmId = frames[f]->landmarkId(im, k);
        if (lmId != 0) {
          landmarks[f].insert(LandmarkId(lmId));
        }
      }
    }
  }

  // find matches
  std::set<LandmarkId> matches;
  std::set_intersection(
      landmarks[0].begin(), landmarks[0].end(),landmarks[1].begin(),
      landmarks[1].end(), std::inserter(matches,matches.begin()));

  // without matches there will be no overlap
  if(matches.size() == 0) {
    return 0.0;
  }

  // draw match images
  for(size_t f=0; f<2; ++f) {
    for (size_t im = 0; im < frames[f]->numFrames(); ++im) {
      cv::KeyPoint keypoint;
      const size_t num = frames[f]->numKeypoints(im);
      const int rows = frames[f]->image(im).rows/10;
      const int cols = frames[f]->image(im).cols/10;
      const double radius = double(std::min(rows,cols))*kptradius_;
      for (size_t k = 0; k < num; ++k) {
        frames[f]->getCvKeypoint(im, k, keypoint);
        if (matches.count(LandmarkId(frames[f]->landmarkId(im, k)))) {
          cv::circle(matchesImg[f].at(im), keypoint.pt*0.1, int(radius), cv::Scalar(255),
                     cv::FILLED);
        }
      }
    }
  }

  // IoU
  double overlap[2];
  for(size_t f=0; f<2; ++f) {
    int intersectionCount = 0;
    int unionCount = 0;
    for (size_t im = 0; im < frames[f]->numFrames(); ++im) {
      cv::Mat intersectionMask, unionMask;
      cv::bitwise_and(matchesImg[f].at(im), detectionsImg[f].at(im), intersectionMask);
      cv::bitwise_or(matchesImg[f].at(im), detectionsImg[f].at(im), unionMask);
      intersectionCount += cv::countNonZero(intersectionMask);
      unionCount += cv::countNonZero(unionMask);
    }
    overlap[f] = double(intersectionCount)/double(unionCount);
  }

  return std::min(overlap[0], overlap[1]);
}

}  // namespace okvis
