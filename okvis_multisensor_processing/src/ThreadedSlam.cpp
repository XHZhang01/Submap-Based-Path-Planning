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
 *  Created on: Aug 21, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file ThreadedSlam.cpp
 * @brief Source file for the ThreadedSlam3 class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <map>
#include <fstream>
#include <iomanip>

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <glog/logging.h>

#include <okvis/ThreadedSlam.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/ceres/ImuError.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis
{

static const int cameraInputQueueSize = 2;

// overlap of imu data before and after two consecutive frames [seconds]:
static const double imuTemporalOverlap = 0.02;


// Constructor.
ThreadedSlam::ThreadedSlam(ViParameters &parameters, std::string dBowDir) :
  visualizer_(parameters),
  hasStarted_(false),
  frontend_(parameters.nCameraSystem.numCameras(), dBowDir),
  parameters_(parameters)
{
  setBlocking(false);
  setRealtimePropagation(parameters_.output.publish_imu_propagated_state);
  init();
}

// Initialises settings and calls startThreads().
void ThreadedSlam::init()
{
  assert(parameters_.nCameraSystem.numCameras() > 0);
  const size_t numCameras = parameters_.nCameraSystem.numCameras();
  shutdown_ = false;

  // setup frontend
  frontend_.setBriskDetectionOctaves(size_t(parameters_.frontend.octaves));
  frontend_.setBriskDetectionThreshold(parameters_.frontend.detection_threshold);
  frontend_.setBriskDetectionAbsoluteThreshold(parameters_.frontend.absolute_threshold);
  frontend_.setBriskMatchingThreshold(parameters_.frontend.matching_threshold);
  frontend_.setBriskDetectionMaximumKeypoints(size_t(parameters_.frontend.max_num_keypoints));
  frontend_.setKeyframeInsertionOverlapThreshold(float(parameters_.frontend.keyframe_overlap));

  // setup estimator
  estimator_.addImu(parameters_.imu);
  for (size_t im = 0; im < numCameras; ++im) {
    // parameters_.camera_extrinsics is never set (default 0's)...
    // do they ever change?
    estimator_.addCamera(parameters_.camera);
  }
  estimator_.addGps(parameters_.gps);
  estimator_.setDetectorUniformityRadius(parameters_.frontend.detection_threshold);

  // time limit if requested
  if(parameters_.estimator.enforce_realtime) {
    estimator_.setOptimisationTimeLimit(
          parameters_.estimator.realtime_time_limit,
          parameters_.estimator.realtime_min_iterations);
  }

  startThreads();
}

// Start all threads.
void ThreadedSlam::startThreads()
{

  // consumer threads
  imuConsumerThread_ = std::thread(&ThreadedSlam::imuConsumerLoop, this);

  // visualisation
  if(parameters_.output.display_matches) {
    visualisationThread_ = std::thread(&ThreadedSlam::visualisationLoop, this);
  }

  //setMainThreadPriority(SCHED_RR, 99);

  // publishing
  publishingThread_ = std::thread(&ThreadedSlam::publishingLoop, this);

}

// Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
ThreadedSlam::~ThreadedSlam()
{

  // shutdown and join threads
  stopThreading();
}

// Add a new image.
bool ThreadedSlam::addImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images)
{
  const size_t numCameras = parameters_.nCameraSystem.numCameras();
  OKVIS_ASSERT_TRUE(Exception, images.size() == numCameras, "wrong number of images provided")

  // assemble frame
  std::vector<okvis::CameraMeasurement> frames(numCameras);
  for(size_t i=0; i<numCameras; ++i) {
    frames.at(i).measurement.image = images.at(i);
    // correct timestamp offset:
    frames.at(i).timeStamp = stamp -okvis::Duration(parameters_.camera.image_delay);
    frames.at(i).sensorId = int(i);
    frames.at(i).measurement.deliversKeypoints = false;
  }

  if (blocking_)
  {
    return cameraMeasurementsReceived_.PushBlockingIfFull(frames,1);
  }
  else
  {
    if(cameraMeasurementsReceived_.PushNonBlockingDroppingIfFull(frames, cameraInputQueueSize)) {
      LOG(WARNING) << "frame drop ";
      return false;
    }
    return true;
  }
}

// Add an IMU measurement.
bool ThreadedSlam::addImuMeasurement(const okvis::Time& stamp,
                                     const Eigen::Vector3d& alpha,
                                     const Eigen::Vector3d& omega)
{

  okvis::ImuMeasurement imu_measurement;
  imu_measurement.measurement.accelerometers.x() = parameters_.imu.s_a.x() * alpha.x();
  imu_measurement.measurement.accelerometers.y() = parameters_.imu.s_a.y() * alpha.y();
  imu_measurement.measurement.accelerometers.z() = parameters_.imu.s_a.z() * alpha.z();
  imu_measurement.measurement.gyroscopes = omega;
  imu_measurement.timeStamp = stamp;

  const int imuQueueSize = 5000;

  if(realtimePropagation_) {
     imuMeasurementsReceivedPropagate_.PushNonBlockingDroppingIfFull(
           imu_measurement, size_t(imuQueueSize));
  }

  if (blocking_)
  {
    return imuMeasurementsReceived_.PushBlockingIfFull(imu_measurement, size_t(imuQueueSize));
  }
  else
  {
    if(imuMeasurementsReceived_.PushNonBlockingDroppingIfFull(
         imu_measurement, size_t(imuQueueSize))) {
      LOG(WARNING) << "imu measurement drop ";
      return false;
    }
    return true;
  }

}

// Add a LiDAR measurement.
bool ThreadedSlam::addLidarMeasurement(const okvis::Time &stamp,
                                       const Eigen::Vector3d &rayMeasurement)
{

  okvis::LidarMeasurement lidarMeasurement;
  lidarMeasurement.measurement.rayMeasurement = rayMeasurement;
  lidarMeasurement.measurement.intensity = 0;
  lidarMeasurement.timeStamp = stamp;

  const int lidarQueueSize = 500000;

  bool drop = false;

  if (blocking_)
  {
    LOG(WARNING) << "lidar measurement blocking ";
    return lidarMeasurementsReceived_.PushBlockingIfFull(lidarMeasurement, size_t(lidarQueueSize));
  }
  else
  {
    drop = lidarMeasurementsReceived_.PushNonBlockingDroppingIfFull(
            lidarMeasurement, size_t(lidarQueueSize));
    if(drop)
      LOG(WARNING) << "lidar measurement drop ";
  }

  return drop;

}
// Add a GPS measurement.
bool ThreadedSlam::addGpsMeasurement(const okvis::Time& stamp,
                                     const Eigen::Vector3d& pos,
                                     const Eigen::Vector3d& err)
{
  okvis::GpsMeasurement  gps_measurement;
  okvis::GpsSensorReadings gpsReading(pos, err(0), err(1), err(2));
  gps_measurement.timeStamp = stamp;
  gps_measurement.measurement = gpsReading;
  const int gpsQueueSize = 5000;

  if (blocking_)
  {
    return gpsMeasurementsReceived_.PushBlockingIfFull(gps_measurement, size_t(gpsQueueSize));
  }
  else
  {
    if(gpsMeasurementsReceived_.PushNonBlockingDroppingIfFull(gps_measurement, size_t(gpsQueueSize))) {
      LOG(WARNING) << "gps measurement drop ";
      return false;
    }

    return true;
  }

}

// Add a GPS measurement (geodetic input).
bool ThreadedSlam::addGeodeticGpsMeasurement(const okvis::Time& stamp,
                                             double lat, double lon, double height,
                                             double hAcc, double vAcc)
{
  if(hAcc > 0.04 || vAcc > 0.04) {
    return false;
  }

  okvis::GpsMeasurement  gps_measurement;
  okvis::GpsSensorReadings gpsReading(lat, lon, height, hAcc, vAcc);
  gps_measurement.timeStamp = stamp;
  gps_measurement.measurement = gpsReading;
  const int gpsQueueSize = 5000;

  if (blocking_)
  {
    return gpsMeasurementsReceived_.PushBlockingIfFull(gps_measurement, size_t(gpsQueueSize));
  }
  else
  {
    if(gpsMeasurementsReceived_.PushNonBlockingDroppingIfFull(gps_measurement, size_t(gpsQueueSize))) {
      LOG(WARNING) << "gps measurement drop ";
      return false;
    }

    return true;
  }

}

// Add Submap alignment constraints to estimator
bool ThreadedSlam::addSubmapAlignmentConstraints(const se::OccupancyMap<se::Res::Multi>* submap_ptr,
                                                 const uint64_t& frame_A_id, const uint64_t& frame_B_id,
                                                 std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& pointCloud) {


  submapAlignmentFactorsReceived_.PushNonBlocking(AlignmentTerm(submap_ptr,frame_A_id,frame_B_id, pointCloud));
  std::cout << "Pushing to the submapAlignmentFactorsReceived_ queue" << std::endl;
  return true;


  //return estimator_.addSubmapAlignmentConstraints(submap_ptr, frame_A_id, frame_B_id, pointCloud);
}

// Set the blocking variable that indicates whether the addMeasurement() functions
// should return immediately (blocking=false), or only when the processing is complete.
void ThreadedSlam::setBlocking(bool blocking)
{
  blocking_ = blocking;
  // disable time limit for optimization
  if(blocking_)
  {
    /// \todo Lock estimator
    //estimator_.setOptimizationTimeLimit(-1.0,parameters_.optimization.max_iterations);
  }
}

bool ThreadedSlam::getNextFrame(MultiFramePtr &multiFrame) {
  std::vector<okvis::CameraMeasurement> frames;
  if(!cameraMeasurementsReceived_.getCopyOfFront(&frames)) {
    return false;
  }
  const size_t numCameras = frames.size();
  multiFrame.reset(new okvis::MultiFrame(parameters_.nCameraSystem, frames.at(0).timeStamp, 0));
  for(size_t im = 0; im < numCameras; ++im)
  {
    cv::Mat filtered = frames.at(im).measurement.image;
    multiFrame->setImage(im, filtered);
    multiFrame->setDepthImage(im, frames.at(im).measurement.depthImage);
  }
  return true;
}

bool ThreadedSlam::processFrame() {
  MultiFramePtr multiFrame;
  ImuMeasurement imuMeasurement;
  LidarMeasurement lidarMeasurement;
  GpsMeasurement gpsMeasurement;
  const size_t numCameras = parameters_.nCameraSystem.numCameras();

  kinematics::Transformation T_WS;
  SpeedAndBias speedAndBias;

  bool ranDetection = false;
  if(firstFrame_) {
    // in the very beginning, we need to wait for the IMU
    if(!imuMeasurementsReceived_.getCopyOfFront(&imuMeasurement)) {
      return false;
    }

    // now get the frames (synchronised timestamp)
    if(!getNextFrame(multiFrame)) {
      return false;
    }

    if(multiFrame->timestamp()-Duration(imuTemporalOverlap) <= imuMeasurement.timeStamp) {
      // that's bad, we have frames without IMU measurements. Discard too old frames
      LOG(WARNING) << "startup: dropping frame because IMU measurements are newer, t="
                   << imuMeasurement.timeStamp;
      std::vector<okvis::CameraMeasurement> frames;
      cameraMeasurementsReceived_.PopBlocking(&frames);
      return false;
    }
    
    // also make sure we have enough IMU measuerements
    if(!imuMeasurementsReceived_.getCopyOfBack(&imuMeasurement)) {
      return false;
    }
    if(imuMeasurement.timeStamp < multiFrame->timestamp() + Duration(imuTemporalOverlap)) {
      return false; // wait for more IMU measurements
    }

    // now get all relevant IMU measurements we have received thus far
    do {
      if(imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement))
      {
        imuMeasurementDeque_.push_back(imuMeasurement);
      } else {
        return false;
      }
    } while(imuMeasurementDeque_.back().timeStamp
            < multiFrame->timestamp() + Duration(imuTemporalOverlap));

    // Drop LiDAR MEasurements before first frame ToDo: is this necessary?
    while(!lidarMeasurementsReceived_.Empty()
      && lidarMeasurementsReceived_.queue_.front().timeStamp < multiFrame->timestamp()){
      lidarMeasurementsReceived_.PopBlocking(&lidarMeasurement);
    }

    // Drop GPS Measurements that are older than the first frame (= first state)
    while(!gpsMeasurementsReceived_.Empty() && gpsMeasurementsReceived_.queue_.front().timeStamp < multiFrame ->timestamp()){
        gpsMeasurementsReceived_.PopBlocking(&gpsMeasurement);
    } // nothing else to do here for GPS

    firstFrame_ = false;
  } else {
    // wait for next frame
    if(!getNextFrame(multiFrame)) {
      if(optimisationThread_.joinable()) {
        // in the very beginning, we can't join because it was not started
        optimisationThread_.join();
      }

      return false;
    }
    // now get all relevant IMU measurements we have received thus far
    while(!shutdown_ && imuMeasurementDeque_.back().timeStamp <
          multiFrame->timestamp() + Duration(imuTemporalOverlap))
    {
      if(imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement))
      {
        imuMeasurementDeque_.push_back(imuMeasurement);
      } else {
        return false;
      }
    }
  }
  if (estimator_.numFrames()==0) {
    // initial state
    bool success = ceres::ImuError::initPose(imuMeasurementDeque_, T_WS);
    OKVIS_ASSERT_TRUE_DBG(Exception, success,
        "pose could not be initialized from imu measurements.")
    (void)(success); // avoid warning on unused variable

    // detection -- needed to check if we can start up
    TimerSwitchable detectTimer("1 DetectAndDescribe");
    if(parameters_.frontend.parallelise_detection) {
      std::vector<std::shared_ptr<std::thread>> detectionThreads;
      for(size_t im = 1; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        detectionThreads.emplace_back(
              new std::thread(&Frontend::detectAndDescribe, &frontend_,
                              im,  multiFrame, T_WC, nullptr));
      }
      kinematics::Transformation T_WC0 = T_WS * (*parameters_.nCameraSystem.T_SC(0));
      frontend_.detectAndDescribe(0, multiFrame, T_WC0, nullptr);
      for(size_t i = 0; i < detectionThreads.size(); ++i) {
        detectionThreads[i]->join();
      }
    } else {
      for(size_t im = 0; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        frontend_.detectAndDescribe(im,  multiFrame, T_WC, nullptr);
      }
    }
    if(!frontend_.isInitialized() && multiFrame->numKeypoints() < 15) {
      LOG(WARNING) << "Not enough keypoints (" << multiFrame->numKeypoints()
                   << ") -- cannot initialise yet.";
      std::vector<okvis::CameraMeasurement> frames;
      cameraMeasurementsReceived_.PopBlocking(&frames);
      return false;
    }
    ranDetection = true;
    detectTimer.stop();

  } else {

    // propagate to have a sensible pose estimate (as needed for detection)
    const StateId currentId = estimator_.currentStateId();
    T_WS = estimator_.pose(currentId);
    speedAndBias = estimator_.speedAndBias(currentId);
    ceres::ImuError::propagation(imuMeasurementDeque_, parameters_.imu, T_WS, speedAndBias,
                                 estimator_.timestamp(currentId), multiFrame->timestamp());
    // now also get all relevant GPS measurements received thus far
    while(!shutdown_ && !gpsMeasurementsReceived_.Empty() && gpsMeasurementsReceived_.queue_.front().timeStamp < multiFrame->timestamp())
    {
        if(gpsMeasurementsReceived_.PopBlocking(&gpsMeasurement))
        {
          gpsMeasurementDeque_.push_back(gpsMeasurement);
        }
    }
  }
  // success, frame processed. Need to remove from the queue, as getNextFrame only obtains a copy.
  std::vector<okvis::CameraMeasurement> frames;
  cameraMeasurementsReceived_.PopBlocking(&frames);

  // detection
  if(!ranDetection) {
    TimerSwitchable detectTimer("1 DetectAndDescribe");
    if(parameters_.frontend.parallelise_detection) {
      std::vector<std::shared_ptr<std::thread>> detectionThreads;
      for(size_t im = 1; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        detectionThreads.emplace_back(
              new std::thread(&Frontend::detectAndDescribe, &frontend_,
                              im,  multiFrame, T_WC, nullptr));
      }
      kinematics::Transformation T_WC0 = T_WS * (*parameters_.nCameraSystem.T_SC(0));
      frontend_.detectAndDescribe(0, multiFrame, T_WC0, nullptr);
      for(size_t i = 0; i < detectionThreads.size(); ++i) {
        detectionThreads[i]->join();
      }
    } else {
      for(size_t im = 0; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        frontend_.detectAndDescribe(im,  multiFrame, T_WC, nullptr);
      }
    }
    if(!frontend_.isInitialized() && multiFrame->numKeypoints() < 15) {
      LOG(WARNING) << "Not enough keypoints (" << multiFrame->numKeypoints()
                   << ") -- cannot initialise yet.";
      return true;
    }
    detectTimer.stop();
  }

  // IMPORTANT: the matcher needs the optimiser to be finished:
  if(optimisationThread_.joinable()) {
    // in the very beginning, we can't join because it was not started
    optimisationThread_.join();
  }

  // break here since all local threads joined
  if(shutdown_) {
    return false;
  }

  // start the matching
  Time matchingStart = Time::now();
  TimerSwitchable matchTimer("2 Match");
  bool asKeyframe = false;
  if (!estimator_.addStates(multiFrame, imuMeasurementDeque_, asKeyframe)) {
    LOG(ERROR)<< "Failed to add state! will drop multiframe.";
    matchTimer.stop();
    return true;
  }
  else{ // Try to add gps measurements
    estimator_.addGpsMeasurementsOnAllGraphs(gpsMeasurementDeque_, imuMeasurementDeque_);
  }
  imuMeasurementsByFrame_[StateId(multiFrame->id())] = imuMeasurementDeque_;

  // call the matcher
  if(!frontend_.dataAssociationAndInitialization(
        estimator_, parameters_, multiFrame, &asKeyframe) && !frontend_.isInitialized()) {
    LOG(WARNING) << "Not enough matches, cannot initialise";
    frontend_.clear();
    estimator_.clear();
    return false;
  }
  estimator_.setKeyframe(StateId(multiFrame->id()), asKeyframe);

  // remove imuMeasurements from deque
  while(!shutdown_ && imuMeasurementDeque_.front().timeStamp <
        multiFrame->timestamp() - Duration(imuTemporalOverlap))
  {
    imuMeasurementDeque_.pop_front();
  }
  matchTimer.stop();

  // remove gpsMeasurements from deque
  while(!shutdown_ && !gpsMeasurementDeque_.empty() && gpsMeasurementDeque_.front().timeStamp < multiFrame->timestamp() )
  {
    gpsMeasurementDeque_.pop_front();
  }

  // add lidar alignment terms
  AlignmentTerm alignmentTerm;
  while(!shutdown_ && !submapAlignmentFactorsReceived_.Empty())
  {
    submapAlignmentFactorsReceived_.PopNonBlocking(&alignmentTerm);
    estimator_.addSubmapAlignmentConstraints(alignmentTerm.submap_ptr, alignmentTerm.frame_A_id,
                                             alignmentTerm.frame_B_id, alignmentTerm.pointCloud_B);
  }


  // start optimise, publish&visualise, marginalise:
  Eigen::Vector3d gyr(0,0,0);
  auto riter = imuMeasurementDeque_.rbegin();
  while(riter!=imuMeasurementDeque_.rend() && riter->timeStamp > multiFrame->timestamp()) {
    gyr = riter->measurement.gyroscopes;
    ++riter;
  }
  Time now = Time::now();
  double dt = parameters_.estimator.realtime_time_limit-(now-matchingStart).toSec();
  if(dt < 0.0) {
    dt = 0.01;
  }
  if(parameters_.estimator.enforce_realtime) {
    estimator_.setOptimisationTimeLimit(dt,
          parameters_.estimator.realtime_min_iterations);
  }
  optimisationThread_ = std::thread(&ThreadedSlam::optimisePublishMarginalise, this, multiFrame, gyr);

  // kick off posegraph optimisation, if needed, too...
  if(estimator_.needsFullGraphOptimisation()) {
    if(fullGraphOptimisationThread_.joinable()) {
      fullGraphOptimisationThread_.join();
    }
    //std::cout << "launching full graph optimisation" << std::endl;
    // hack: call full graph optimisation
    fullGraphOptimisationThread_ = std::thread(
          &ViSlamBackend::optimiseFullGraph, &estimator_,
          parameters_.estimator.full_graph_iterations,
          std::ref(posegraphOptimisationSummary_),
          parameters_.estimator.full_graph_num_threads, false);
  }

  return true;
}

void ThreadedSlam::optimisePublishMarginalise(MultiFramePtr multiFrame,
                                              const Eigen::Vector3d& gyroReading) {
  kinematics::Transformation T_WS;
  SpeedAndBias speedAndBiases;
  const size_t numCameras = parameters_.nCameraSystem.numCameras();

  // optimise (if initialised)
  TimerSwitchable optimiseTimer("3 Optimise");
  std::vector<StateId> updatedStatesRealtime;
  estimator_.optimiseRealtimeGraph(
        parameters_.estimator.realtime_max_iterations, updatedStatesRealtime,
        parameters_.estimator.realtime_num_threads);

  optimiseTimer.stop();

  // import pose graph optimisation
  std::vector<StateId> updatedStatesSync;
  if(estimator_.isLoopClosureAvailable()) {
    OKVIS_ASSERT_TRUE(Exception, !estimator_.isLoopClosing(), "bug")
    TimerSwitchable synchronisationTimer("5 Import full optimisation");
    estimator_.synchroniseRealtimeAndFullGraph(updatedStatesSync);
    synchronisationTimer.stop();
  }

  // prepare for publishing
  TimerSwitchable publishTimer("4 Prepare publishing");
  T_WS = estimator_.pose(StateId(multiFrame->id()));
  speedAndBiases = estimator_.speedAndBias(StateId(multiFrame->id()));
  StateId id(multiFrame->id());
  State state;
  state.id = id;
  state.T_WS = T_WS;
  state.v_W = speedAndBiases.head<3>();
  state.b_g = speedAndBiases.segment<3>(3);
  state.b_a = speedAndBiases.tail<3>();
  state.omega_S = gyroReading - speedAndBiases.segment<3>(3);
  state.timestamp = multiFrame->timestamp();
  state.previousImuMeasurements = imuMeasurementsByFrame_.at(id);
  state.isKeyframe = estimator_.isKeyframe(id);
  TrackingState trackingState;
  trackingState.id = id;
  trackingState.isKeyframe = estimator_.isKeyframe(id);
  trackingState.recognisedPlace = estimator_.closedLoop(id);
  const double trackingQuality = estimator_.trackingQuality(id);
  if(trackingQuality < 0.01) {
    trackingState.trackingQuality = TrackingQuality::Lost;
  } else if (trackingQuality < 0.3){
    trackingState.trackingQuality = TrackingQuality::Marginal;
  } else {
    trackingState.trackingQuality = TrackingQuality::Good;
  }
  trackingState.currentKeyframeId = estimator_.mostOverlappedStateId(id, false);

  // re-propagate
  hasStarted_.store(true);

  // now publish
  if(optimisedGraphCallback_) {
    // current state & tracking info via State and Tracking State.
    // the graph:
    std::vector<StateId> updatedStateIds;
    PublicationData publicationData;
    publicationData.state = state;
    publicationData.trackingState = trackingState;
    publicationData.updatedStates.reset(new AlignedMap<StateId, State>());
    if(updatedStatesSync.size()>0) {
      updatedStateIds = updatedStatesSync;
    } else {
      updatedStateIds = updatedStatesRealtime;
    }
    for(const auto & id : updatedStateIds) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      const bool isKeyframe = estimator_.isKeyframe(id);
      ImuMeasurementDeque imuMeasurements = imuMeasurementsByFrame_.at(id);
      Eigen::Vector3d omega_S(0.0, 0.0, 0.0); // get this for real now:
      for(auto riter = imuMeasurements.rbegin(); riter!=imuMeasurements.rend(); ++riter) {
        if(riter->timeStamp < timestamp) {
          omega_S = riter->measurement.gyroscopes - speedAndBias.segment<3>(3);
          break;
        }
      }
      (*publicationData.updatedStates)[id] = State{T_WS, speedAndBias.head<3>(),
                                    speedAndBias.segment<3>(3), speedAndBias.tail<3>(),
                                    omega_S, timestamp, id, imuMeasurements, isKeyframe};
    }

    // landmarks:
    publicationData.landmarksPublish.reset(new MapPointVector());
    MapPoints landmarks;
    estimator_.getLandmarks(landmarks);
    publicationData.landmarksPublish->reserve(landmarks.size());
    for(const auto & lm : landmarks) {
      publicationData.landmarksPublish->push_back(
            MapPoint(lm.first.value(), lm.second.point, lm.second.quality));
    }
    
    // now publish in separate thread. queue size 3 to ensure nothing ever lost.
    if(blocking_){
      // in blocked processing, we also want to be able to block the okvis-estimator from outside
      publicationQueue_.PushBlockingIfFull(publicationData,1);
    }
    else{
      const bool overrun = publicationQueue_.PushNonBlockingDroppingIfFull(publicationData,3);
      if(overrun) {
        LOG(ERROR) << "publication (full update) overrun: dropping";
      }
    }

    if(realtimePropagation_) {
      // pass on into IMU processing loop for realtime propagation later.
      // queue size 1 because we can afford to lose them; re-prop. just needs newest stuff.
      const bool overrun2 = lastOptimisedQueue_.PushNonBlockingDroppingIfFull(publicationData,3);
      if(overrun2) {
        LOG(WARNING) << "publication (full update) overrun 2: dropping";
      }
    }

    // Update Realtime Trajectory object
    std::set<StateId> affectedStateIds;
    trajectory_.update(publicationData.trackingState, publicationData.updatedStates, affectedStateIds);
  }
  publishTimer.stop();

  // visualise
  TimerSwitchable visualisationTimer("6 Visualising");
  if(parameters_.output.display_overhead) {
    TimerSwitchable visualisation1Timer("6.1 Visualising overhead");
    // draw debug overhead image
    cv::Mat image(580, 580, CV_8UC3);
    image.setTo(cv::Scalar(10, 10, 10));
    estimator_.drawOverheadImage(image);
    overheadImages_.PushNonBlockingDroppingIfFull(image,1);
    visualisation1Timer.stop();
  }
  if(parameters_.output.display_matches) {
    TimerSwitchable visualisation2Timer("6.2 Prepare visualising matches");
    // draw matches
    // fill in information that requires access to estimator.
    ViVisualizer::VisualizationData::Ptr visualizationDataPtr(
          new ViVisualizer::VisualizationData());
    visualizationDataPtr->observations.resize(multiFrame->numKeypoints());
    okvis::MapPoint2 landmark;
    okvis::ObservationVector::iterator it = visualizationDataPtr
        ->observations.begin();
    for (size_t camIndex = 0; camIndex < numCameras; ++camIndex) {
      for (size_t k = 0; k < multiFrame->numKeypoints(camIndex); ++k) {
        OKVIS_ASSERT_TRUE_DBG(Exception,it != visualizationDataPtr->observations.end(),
                              "Observation-vector not big enough")
        it->keypointIdx = k;
        multiFrame->getKeypoint(camIndex, k, it->keypointMeasurement);
        multiFrame->getKeypointSize(camIndex, k, it->keypointSize);
        it->cameraIdx = camIndex;
        it->frameId = multiFrame->id();
        it->landmarkId = multiFrame->landmarkId(camIndex, k);
        if (estimator_.isLandmarkAdded(LandmarkId(it->landmarkId)) &&
            estimator_.isObserved(KeypointIdentifier(it->frameId, camIndex, k))) {
          estimator_.getLandmark(LandmarkId(it->landmarkId), landmark);
          it->landmark_W = landmark.point;
          it->classification = landmark.classification;
          if (estimator_.isLandmarkInitialised(LandmarkId(it->landmarkId)))
            it->isInitialized = true;
          else
            it->isInitialized = false;
        } else {
          it->landmark_W = Eigen::Vector4d(0, 0, 0, 0);
          // set to infinity to tell visualizer that landmark is not added...
        }
        ++it;
      }
    }
    visualizationDataPtr->T_WS = estimator_.pose(estimator_.currentStateId());
    visualizationDataPtr->currentFrames = multiFrame;
    visualizationDataPtr->isKeyframe = trackingState.isKeyframe;
    visualizationDataPtr->recognisedPlace = trackingState.recognisedPlace;
    if(trackingState.trackingQuality == TrackingQuality::Lost) {
      visualizationDataPtr->trackingQuality =
          ViVisualizer::VisualizationData::TrackingQuality::Lost;
    } else if(trackingState.trackingQuality == TrackingQuality::Marginal) {
      visualizationDataPtr->trackingQuality =
          ViVisualizer::VisualizationData::TrackingQuality::Marginal;
    }
    visualisation2Timer.stop();
    visualisationData_.PushNonBlockingDroppingIfFull(visualizationDataPtr,1);
  }
  visualisationTimer.stop();

  // apply marginalisation strategy
  bool expand = true;
  TimerSwitchable marginaliseTimer("7 Marginalise");
  estimator_.applyStrategy(
        size_t(parameters_.estimator.num_keyframes),
        size_t(parameters_.estimator.num_loop_closure_frames),
        size_t(parameters_.estimator.num_imu_frames), expand);
  marginaliseTimer.stop();
}

// Loop to process IMU measurements.
void ThreadedSlam::imuConsumerLoop()
{
  ImuMeasurementDeque imuMeasurements;
  ImuMeasurement imuMeasurement;
  kinematics::Transformation T_WS;
  SpeedAndBias speedAndBiases;
  Time time;
  PublicationData publicationData;
  publicationData.state.timestamp = Time(0);
  
  Time lastPublicationTime(0);
  while (!shutdown_)
  {
    if(!realtimePropagation_) {
      continue;
    }
    // get data and check for termination request
    if(!imuMeasurementsReceivedPropagate_.PopBlocking(&imuMeasurement)) {
      return;
    }
    imuMeasurements.push_back(imuMeasurement);
    if(hasStarted_) {
      if(lastOptimisedQueue_.PopNonBlocking(&publicationData)) {

        // now re-propagate
        T_WS = publicationData.state.T_WS;
        speedAndBiases.head<3>() = publicationData.state.v_W;
        speedAndBiases.segment<3>(3) = publicationData.state.b_g;
        speedAndBiases.tail<3>() = publicationData.state.b_a;
        time = publicationData.state.timestamp;
        ceres::ImuError::propagation(imuMeasurements, parameters_.imu, T_WS, speedAndBiases,
                                     time, imuMeasurement.timeStamp);
        while(imuMeasurements.front().timeStamp < time - Duration(imuTemporalOverlap))
        {
          imuMeasurements.pop_front(); // delete unnecessary IMU measurements
        }
        time = imuMeasurement.timeStamp;
      } else {
        if(publicationData.state.timestamp.sec == Time(0).sec) {
          continue;
        }

        // this should now in principle only be one integration step.
        ceres::ImuError::propagation(imuMeasurements, parameters_.imu, T_WS, speedAndBiases,
                                     time, imuMeasurement.timeStamp);
        time = imuMeasurement.timeStamp;
      }
      // no more updating. Careful: do not modify, but create new instances
      publicationData.updatedStates.reset(new AlignedMap<StateId, State>());
      publicationData.landmarksPublish.reset(new MapPointVector());

      // now the actual publishing
      if(optimisedGraphCallback_) {
        State state;
        // leave id as invalid, since there is no frame associated
        state.T_WS = T_WS;
        state.v_W = speedAndBiases.head<3>();
        state.b_g = speedAndBiases.segment<3>(3);
        state.b_a = speedAndBiases.tail<3>();
        state.omega_S = imuMeasurement.measurement.gyroscopes - speedAndBiases.segment<3>(3);
        state.timestamp = time;
        state.isKeyframe = false;
        publicationData.state = state;

        // push for publication according to the set rate...
        if((publicationData.state.timestamp - lastPublicationTime).toSec()
              < (1.0/parameters_.output.imu_propagated_state_publishing_rate)) {
          continue; // don't publish...
        }
        if(publicationQueue_.Size()>1) {
          LOG(WARNING) << "publication overrun: dropping";
        } else {
          publicationQueue_.PushNonBlocking(publicationData);
          lastPublicationTime = publicationData.state.timestamp;
        }

      }
    }
  }
}

// Loop to process visualisations.
void ThreadedSlam::visualisationLoop()
{
  while (!shutdown_) {
    ViVisualizer::VisualizationData::Ptr visualisationData;
    if(visualisationData_.PopBlocking(&visualisationData)) {
      std::vector<cv::Mat> outImages(parameters_.nCameraSystem.numCameras());
      for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
        outImages[i] = visualizer_.drawMatches(visualisationData, i);
      }
      visualisationImages_.PushNonBlockingDroppingIfFull(outImages,1);
    } else {
      return;
    }
  }
}

// Loop to process publishing.
void ThreadedSlam::publishingLoop()
{
  while (!shutdown_) {
    PublicationData publicationData;
    if(publicationQueue_.PopBlocking(&publicationData)) {
      if(optimisedGraphCallback_) {
        optimisedGraphCallback_(publicationData.state, publicationData.trackingState,
                                publicationData.updatedStates, publicationData.landmarksPublish);
      }
    } else {
      return;
    }
  }
}

// trigger display (needed because OSX won't allow threaded display)
void ThreadedSlam::display(cv::Mat& images, cv::Mat& topDebugImg)
{

  std::vector<cv::Mat> outImages(parameters_.nCameraSystem.numCameras());
  if(visualisationImages_.PopNonBlocking(&outImages)) {
    // draw
    for (size_t im = 0; im < parameters_.nCameraSystem.numCameras(); im++) {
      if(im==0) {
        images = outImages[0];
      } else {
        cv::hconcat(images, outImages[im], images);
      }
    }
  }

  // top view
  overheadImages_.PopNonBlocking(&topDebugImg);
}

void ThreadedSlam::stopThreading() {
  frontend_.endCnnThreads();
  // thread safety -- join running stuff
  imuMeasurementsReceived_.Shutdown();
  cameraMeasurementsReceived_.Shutdown();
  imuMeasurementsReceivedPropagate_.Shutdown();
  gpsMeasurementsReceived_.Shutdown();
  lidarMeasurementsReceived_.Shutdown();
  submapAlignmentFactorsReceived_.Shutdown();
  visualisationImages_.Shutdown();
  visualisationData_.Shutdown();
  publicationQueue_.Shutdown();
  lastOptimisedQueue_.Shutdown();

  shutdown_ = true;
  if(imuConsumerThread_.joinable()) {
    imuConsumerThread_.join();
  }
  if(optimisationThread_.joinable()) {
    optimisationThread_.join();
  }
  if(fullGraphOptimisationThread_.joinable()) {
    fullGraphOptimisationThread_.join();
    // import pose graph optimisation
    if(estimator_.isLoopClosureAvailable()) {
      OKVIS_ASSERT_TRUE(Exception, !estimator_.isLoopClosing(), "bug")
      TimerSwitchable synchronisationTimer("5 Import full optimisation");
      std::vector<StateId> updatedStates;
      estimator_.synchroniseRealtimeAndFullGraph(updatedStates);
      synchronisationTimer.stop();
    }
  }
  if(visualisationThread_.joinable()) {
    visualisationThread_.join();
  }
  if(publishingThread_.joinable()) {
    publishingThread_.join();
  }

  if(estimator_.isLoopClosureAvailable()) {
    OKVIS_ASSERT_TRUE(Exception, !estimator_.isLoopClosing(), "bug")
    std::vector<StateId> updatedStates;
    estimator_.synchroniseRealtimeAndFullGraph(updatedStates);
  }
}

void ThreadedSlam::writeFinalTrajectoryCsv()
{
  // thread safety -- join running stuff
  stopThreading();

  // trajectory writing
  if(!finalTrajectoryCsvFileName_.empty()) {
    estimator_.writeFinalCsvTrajectory(finalTrajectoryCsvFileName_, rpg_);
  }
}

void ThreadedSlam::writeGlobalTrajectoryCsv(const std::string& csvFileName)
{

  estimator_.writeGlobalCsvTrajectory(csvFileName);
}

void ThreadedSlam::doFinalBa()
{
  // thread safety -- join running stuff
  stopThreading();

  // now call it
  estimator_.doFinalBa(1000, posegraphOptimisationSummary_);

  // Update Trajectory Object
  std::vector<StateId> fullGraphStateIds;
  estimator_.getFinalStateList(fullGraphStateIds);
  LOG(INFO) << "updating " << fullGraphStateIds.size() << " states after final BA" << std::endl;

  std::shared_ptr<AlignedMap<StateId, State>> updatedStates(new AlignedMap<StateId, State>());
  for(const auto & id : fullGraphStateIds) {
    kinematics::Transformation T_WS = estimator_.pose(id);
    SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
    Time timestamp = estimator_.timestamp(id);
    ImuMeasurementDeque imuMeasurements = imuMeasurementsByFrame_.at(id);
    Eigen::Vector3d omega_S(0.0, 0.0, 0.0); // get this for real now:
    for(auto riter = imuMeasurements.rbegin(); riter!=imuMeasurements.rend(); ++riter) {
      if(riter->timeStamp < timestamp) {
        omega_S = riter->measurement.gyroscopes - speedAndBias.segment<3>(3);
        break;
      }
    }
    (*updatedStates)[id] = State{T_WS, speedAndBias.head<3>(),
            speedAndBias.segment<3>(3), speedAndBias.tail<3>(),
                    omega_S, timestamp, id, imuMeasurements};
  }
  TrackingState trackingState;
  trackingState.id = fullGraphStateIds.back();
  trackingState.isKeyframe = estimator_.isKeyframe(fullGraphStateIds.back());
  trackingState.recognisedPlace = estimator_.closedLoop(fullGraphStateIds.back());
  const double trackingQuality = estimator_.trackingQuality(fullGraphStateIds.back());
  if(trackingQuality < 0.01) {
    trackingState.trackingQuality = TrackingQuality::Lost;
  } else if (trackingQuality < 0.3){
    trackingState.trackingQuality = TrackingQuality::Marginal;
  } else {
    trackingState.trackingQuality = TrackingQuality::Good;
  }
  trackingState.currentKeyframeId = estimator_.mostOverlappedStateId(fullGraphStateIds.back(), false);

  // Update Realtime Trajectory object
  std::set<StateId> affectedStateIds;
  trajectory_.update(trackingState, updatedStates, affectedStateIds);

  std::shared_ptr<MapPointVector> dummy(new MapPointVector()); // ToDo: How Does this have to be filled?
  optimisedGraphCallback_((*updatedStates)[fullGraphStateIds.back()], trackingState,
                          updatedStates, dummy);
}

bool ThreadedSlam::saveMap() {
  // thread safety -- join running stuff
  stopThreading();

  // now call it
  if(!finalTrajectoryCsvFileName_.empty()) {
    return estimator_.saveMap(mapCsvFileName_);
  }
  return false;
}

void ThreadedSlam::dumpGpsResiduals(const std::string &gpsResCsvFileName)
{
  // thread safety -- join running stuff
  stopThreading();

  // call residual writer on graph
  estimator_.dumpGpsResiduals(gpsResCsvFileName);


}

}  // namespace okvis
