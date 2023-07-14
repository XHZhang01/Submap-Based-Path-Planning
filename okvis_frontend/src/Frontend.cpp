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
 *  Created on: Mar 27, 2015
 *      Author: Andreas Forster (an.forster@gmail.com)
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file Frontend.cpp
 * @brief Source file for the Frontend class.
 * @author Andreas Forster
 * @author Stefan Leutenegger
 */

#include <thread>
#include <iomanip>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>

#include <brisk/brisk.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <glog/logging.h>

// DBoW2
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <DBoW2/DBoW2.h>
#pragma GCC diagnostic pop
#include <DBoW2/FBrisk.hpp>

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

#include <okvis/Frontend.hpp>

// okvis ceres
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/ImuError.hpp>

// cameras and distortions
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>
#include <okvis/triangulation/stereo_triangulation.hpp>
#include <okvis/cameras/EucmCamera.hpp>

// Kneip RANSAC
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/FrameAbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRotationOnlySacProblem.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

static const double kptrad = 0.09;
//cv::Ptr<cv::CLAHE> mClahe;

/// \brief Opaque stuct to use DBoW for loop closure.
struct Frontend::DBoW {
  DBoW(const std::string& dBowVocDir)
      : vocabulary(dBowVocDir+"/small_voc.yml.gz"),
        // false = do not use direct index.
        database(vocabulary, false, 0)
  {
  }

  DBoW2::TemplatedVocabulary<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk> vocabulary; ///< BRISK Vocabulary.
  DBoW2::TemplatedDatabase<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk> database; ///< BRISK Database.
  std::vector<uint64> poseIds; ///< The multiframe IDs corresponding to the dBow ones.

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Constructor.
Frontend::Frontend(size_t numCameras, std::string dBowVocDir)
    : isInitialized_(false),
      numCameras_(numCameras),
      briskDetectionOctaves_(0),
      briskDetectionThreshold_(40.0),
      briskDetectionAbsoluteThreshold_(200.0),
      briskDetectionMaximumKeypoints_(450),
      briskDescriptionRotationInvariance_(true),
      briskDescriptionScaleInvariance_(false),
      briskMatchingThreshold_(60.0),
      keyframeInsertionOverlapThreshold_(0.55f),
      dBow_(new DBoW(dBowVocDir)) {
  // create mutexes for feature detectors and descriptor extractors
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectorMutexes_.push_back(std::unique_ptr<std::mutex>(new std::mutex()));
  }
  trackingLost_ = false;
  initialiseBriskFeatureDetectors();

#ifdef OKVIS_USE_NN
  // Deserialize the ScriptModule from a file using torch::jit::load().
  networks_.resize(numCameras);
  for (size_t i = 0; i < numCameras_; ++i) {
#ifdef OKVIS_USE_GPU
    networks_[i].reset(new Network(torch::jit::load(dBowVocDir+"/fast-scnn.pt", torch::kCUDA)));
    networks_[i]->to(torch::kCUDA);
#else
    networks_[i].reset(new Network(torch::jit::load(dBowVocDir+"/fast-scnn.pt", torch::kCPU)));
    networks_[i]->to(torch::kCPU);
#endif
  }

#endif
}

Frontend::~Frontend() {
  endCnnThreads();
}

// Detection and descriptor extraction on a per image basis.
bool Frontend::detectAndDescribe(size_t cameraIndex, std::shared_ptr<okvis::MultiFrame> frameOut,
                                 const okvis::kinematics::Transformation& T_WC,
                                 const std::vector<cv::KeyPoint>* keypoints) {
  OKVIS_ASSERT_TRUE_DBG(Exception, cameraIndex < numCameras_,
                        "Camera index exceeds number of cameras.")
  std::lock_guard<std::mutex> lock(*featureDetectorMutexes_[cameraIndex]);

  // check there are no keypoints here
  OKVIS_ASSERT_TRUE(Exception, keypoints == nullptr, "external keypoints currently not supported")

  // hack: also initialise those maps for camera-aware extraction
  for (size_t i = 0; i < numCameras_; ++i) {
    if(!std::static_pointer_cast<cv::BriskDescriptorExtractor>(
         descriptorExtractors_.at(i))->isCameraAware()) {
      cv::Mat rays;
      cv::Mat imageJacobians;
      bool success = frameOut->geometry(i)->getCameraAwarenessMaps(rays, imageJacobians);
      OKVIS_ASSERT_TRUE(Exception, success, "camera awareness maps not initialised");
      if (std::strcmp(frameOut->geometry(i)->type().c_str(), "EUCMCamera") == 0){
        std::static_pointer_cast<cv::BriskDescriptorExtractor>(descriptorExtractors_.at(i))->setCameraProperties(
                rays, imageJacobians, float(std::static_pointer_cast<const cameras::EucmCamera>(frameOut->geometry(i))->focalLengthU()));
      }
      else{
        std::static_pointer_cast<cv::BriskDescriptorExtractor>(descriptorExtractors_.at(i))->setCameraProperties(
                rays, imageJacobians, float(std::static_pointer_cast<const cameras::PinholeCameraBase>(frameOut->geometry(i))->focalLengthU()));
      }
    }
  }

  // ExtractionDirection == gravity direction in camera frame
  Eigen::Vector3d g_in_W(0, 0, -1);
  Eigen::Vector3d extractionDir = T_WC.inverse().C() * g_in_W;
  std::static_pointer_cast<cv::BriskDescriptorExtractor>(descriptorExtractors_[cameraIndex])
      ->setExtractionDirection(
        cv::Vec3f(float(extractionDir[0]),float(extractionDir[1]),float(extractionDir[2])));

  frameOut->setDetector(cameraIndex, featureDetectors_[cameraIndex]);
  frameOut->setExtractor(cameraIndex, descriptorExtractors_[cameraIndex]);
#ifdef OKVIS_USE_NN
  frameOut->setNetwork(cameraIndex, networks_[cameraIndex]);
#endif

  // detect
  frameOut->detect(cameraIndex);


  // extract
  frameOut->describe(cameraIndex);


  // precompute backprojections
  frameOut->computeBackProjections(cameraIndex);

  return true;
}

// Matching as well as initialization of landmarks and state.
bool Frontend::dataAssociationAndInitialization(
    Estimator &estimator, const okvis::ViParameters& params,
    std::shared_ptr<okvis::MultiFrame> framesInOut, bool* asKeyframe) {

  // match new keypoints to existing landmarks/keypoints
  // initialise new landmarks (states)
  // outlier rejection by consistency check
  // RANSAC (2D2D / 3D2D)
  // decide keyframe
  // left-right stereo match & init

  // find distortion type
  cameras::NCameraSystem::DistortionType distortionType = params.nCameraSystem.distortionType(0);
  for (size_t i = 1; i < params.nCameraSystem.numCameras(); ++i) {
    OKVIS_ASSERT_TRUE(Exception, distortionType == params.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet")
  }
  int num3dMatches = 0;

  // first frame? (did do addStates before, so 1 frame minimum in estimator)
  double trackingQuality = 1.0;
  if (estimator.numFrames() > 1) {

    // match to all landmarks
    TimerSwitchable matchMapTimer("2.1 match to map");
    switch (distortionType) {
      case okvis::cameras::NCameraSystem::RadialTangential: {
        num3dMatches = matchToMap<cameras::PinholeCamera<cameras::RadialTangentialDistortion>>(
            estimator, params, framesInOut->id());
        break;
      }
      case okvis::cameras::NCameraSystem::Equidistant: {
        num3dMatches = matchToMap<cameras::PinholeCamera<cameras::EquidistantDistortion>>(
            estimator, params, framesInOut->id());
        break;
      }
      case okvis::cameras::NCameraSystem::RadialTangential8: {
        num3dMatches = matchToMap<cameras::PinholeCamera<cameras::RadialTangentialDistortion8>>(
            estimator, params, framesInOut->id());
        break;
      }
      case okvis::cameras::NCameraSystem::NoDistortion: {
        num3dMatches = matchToMap<cameras::EucmCamera>(estimator, params, framesInOut->id());
        break;
      }
      default:
        OKVIS_THROW(Exception, "Unsupported distortion type.")
        break;
    }
    std::vector<StateId> updatedStatesRealtime;
    estimator.optimiseRealtimeGraph(5, updatedStatesRealtime, params.estimator.realtime_num_threads, false, true);
    matchMapTimer.stop();

    // check tracking quality
    trackingQuality = estimator.trackingQuality(StateId(framesInOut->id()));
    if (trackingQuality < 0.01) {
      if(estimator.numFrames() == 2 && params.nCameraSystem.numCameras()==1) {
        // mono. we can't have matches at this point, so don't warn
      } else {
        LOG(WARNING) << "Tracking failure (quality "<< trackingQuality
                     <<"). Number of 3d2d-matches: " << num3dMatches;
      }
    }

    // do motion stereo
    if(!trackingLost_ || !isInitialized_) {
      bool rotationOnly = false;
      TimerSwitchable matchMotionStereoTimer("2.2 match motion stereo");
      switch (distortionType) {
        case okvis::cameras::NCameraSystem::RadialTangential: {
          matchMotionStereo<cameras::PinholeCamera<cameras::RadialTangentialDistortion>>(
              estimator, params, framesInOut->id(), rotationOnly);
          break;
        }
        case okvis::cameras::NCameraSystem::Equidistant: {
          matchMotionStereo<cameras::PinholeCamera<cameras::EquidistantDistortion>>(
              estimator, params, framesInOut->id(), rotationOnly);
          break;
        }
        case okvis::cameras::NCameraSystem::RadialTangential8: {
          matchMotionStereo<cameras::PinholeCamera<cameras::RadialTangentialDistortion8>>(
              estimator, params, framesInOut->id(), rotationOnly);
          break;
        }
        case okvis::cameras::NCameraSystem::NoDistortion: {
          matchMotionStereo<cameras::EucmCamera>(estimator, params, framesInOut->id(), rotationOnly);
          break;
        }
        default:
          OKVIS_THROW(Exception, "Unsupported distortion type.")
          break;
      }
      if(!rotationOnly) {
        if(!isInitialized_) {
          isInitialized_ = true;
          LOG(INFO) << "Initialized!";
        }
      }
      trackingQuality = estimator.trackingQuality(StateId(framesInOut->id()));
      matchMotionStereoTimer.stop();
    }

    // keyframe decision, at the moment only landmarks that match with keyframe are initialised
    *asKeyframe = doWeNeedANewKeyframe(estimator, framesInOut);
  } else {
    *asKeyframe = true;  // first frame needs to be keyframe
  }

  /*LOOP CLOSURES*/
  if(params.estimator.do_loop_closures && !estimator.isLoopClosing()
     && !estimator.isLoopClosureAvailable() && !estimator.needsFullGraphOptimisation()) {

    TimerSwitchable matchDBoWTimer("2.3 loop closure query");
    // first, we are trying to match the database for loop closures
    std::vector<std::vector<uchar>> features(framesInOut->numKeypoints(0));
    for(size_t k=0; k<framesInOut->numKeypoints(0); ++k) {
      features.at(k).resize(48); // TODO: get 48 from feature
      memcpy(features.at(k).data(), framesInOut->keypointDescriptor(0,k), 48*sizeof(uchar));
    }
    DBoW2::QueryResults dBoWResult;
    dBow_->database.query(features,dBoWResult,-1); // get all matches
    // sort ascending -- we want to match oldest...
    std::sort( dBoWResult.begin( ), dBoWResult.end( ),
               [ ]( const DBoW2::Result& lhs, const DBoW2::Result& rhs )
    {
       return lhs.Id < rhs.Id;
    });
    matchDBoWTimer.stop();
    TimerSwitchable matchLoopClosureTimer("2.4 loop closure descriptor matching", true);
    TimerSwitchable ransacLoopClosureTimer("2.5 loop closure ransacking", true);
    TimerSwitchable attemptLoopClosureTimer("2.6 attempt loop closure", true);
    int attempts0 = 0;
    int attempts = 0;

    for(size_t f=0; f < dBoWResult.size() && attempts<10; ++f) {
      // start with oldest keyframe match
      size_t id = dBoWResult.at(f).Id;
      double p = dBoWResult.at(f).Score;

      // get old multiframe
      uint64_t poseId = dBow_->poseIds.at(id);

      // nonmax suppression
      if(f>0){
        if(dBoWResult.at(f-1).Score > p && estimator.isPlaceRecognitionFrame(StateId(poseId))) {
          continue;
        }
        if(f>1) {
          if(dBoWResult.at(f-2).Score > p && estimator.isPlaceRecognitionFrame(StateId(poseId))) {
            continue;
          }
        }
      }
      if(f+1<dBoWResult.size()){
        if(dBoWResult.at(f+1).Score > p && estimator.isPlaceRecognitionFrame(StateId(poseId))) {
          continue;
        }
        if(f+2<dBoWResult.size()) {
          if(dBoWResult.at(f+2).Score > p && estimator.isPlaceRecognitionFrame(StateId(poseId))) {
            continue;
          }
        }
      }

      if(attempts0>40 || attempts>10) break;
      if(p > 0.3) {
        attempts0++;
        const std::shared_ptr<const MultiFrame> oldFrame = estimator.multiFrame(StateId(poseId));
        /// \todo move to separate thread

        // check if already existing loop closure or matching against current frame
        if(!estimator.isPoseGraphFrame(StateId(poseId))) {
          continue;
        }
        if(estimator.isLoopClosureFrame(StateId(poseId))) {
          continue;
        }
        if(estimator.isRecentLoopClosureFrame(StateId(poseId))) {
          continue;
        }
        if(!estimator.isPlaceRecognitionFrame(StateId(poseId))) {
          continue;
        }

        // geometric verification:
        matchLoopClosureTimer.start();
        std::map<LandmarkId, std::vector<const uchar*>> descriptors;
        AlignedMap<LandmarkId, Eigen::Vector4d> landmarks;
        size_t ctr = 0;
        opengv::absolute_pose::LoopclosureNoncentralAbsoluteAdapter::Points points;
        std::map<KeypointIdentifier, uint64_t> matches;

        // find matchable points
        for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
          for(size_t kOld = 0; kOld < oldFrame->numKeypoints(im); ++kOld) {
            uint64_t lmId = oldFrame->landmarkId(im, kOld);
            if(lmId == 0) {
              continue;
            }
            // 1. get the 3D points / descriptors from old frame
            const uchar* oldDescripor = oldFrame->keypointDescriptor(im, kOld);
            Eigen::Vector4d landmark;
            bool isInitialised = false;
            oldFrame->getLandmark(im, kOld, landmark, isInitialised);
            if(!isInitialised) continue;
            if(landmark.norm() < 1.0e-12) {
              continue; // bit of a hack, signals there was no associated 3d point
            }
#ifdef OKVIS_USE_NN
            if(params.frontend.use_cnn && oldFrame->isClassified(im)) {
              // make sure not to use sky or person points here
              cv::Mat classification;
              oldFrame->getClassification(im, kOld, classification);
              if(classification.at<float>(10) > 3.5f) { // Sky
                continue;
              }
              if(classification.at<float>(11) > 53.5f) { // Person
                continue;
              }
            }
#endif
            auto iter = descriptors.find(LandmarkId(lmId));
            if(iter!=descriptors.end()) {
              // just add descriptor
              iter->second.push_back(oldDescripor);
            } else {
              descriptors[LandmarkId(lmId)].push_back(oldDescripor);
              landmarks[LandmarkId(lmId)] = landmark;
            }
          }
        }

        // match
        for(auto iter = landmarks.begin(); iter!= landmarks.end(); ++iter) {
          for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
            const size_t K = framesInOut->numKeypoints(im);
            double distMin = briskMatchingThreshold_;
            size_t kMin = 0;
            for(auto oldDescripor : descriptors.at(iter->first)) {
              for(size_t k = 0; k < K; ++k) {
                const uchar* descripor = framesInOut->keypointDescriptor(im, k);
                const auto dist = brisk::Hamming::PopcntofXORed(descripor, oldDescripor, 3);
                if(dist < distMin) {
                  distMin = dist;
                  kMin = k;
                }
              }
            }
            // now get best match
            if(distMin < briskMatchingThreshold_) {
              ctr++;
              points[iter->first.value()] = iter->second;
              matches[KeypointIdentifier(framesInOut->id(), im, kMin)]=iter->first.value();
            }
          }
        }
        matchLoopClosureTimer.stop();
        if(ctr<10 || points.size() < 8) {
          continue;
        }

        // 3. run 3d2d RANSAC
        ransacLoopClosureTimer.start();
        // create a RelativePoseSac problem and RANSAC
        opengv::absolute_pose::LoopclosureNoncentralAbsoluteAdapter adapter(
              points, matches, params.nCameraSystem, framesInOut);
        typedef opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem<
              opengv::absolute_pose::LoopclosureNoncentralAbsoluteAdapter> LoopclosureAbsoluteModel;
        opengv::sac::Ransac<LoopclosureAbsoluteModel> ransac;
        std::shared_ptr<LoopclosureAbsoluteModel> absposeproblem_ptr(new LoopclosureAbsoluteModel(
            adapter, LoopclosureAbsoluteModel::Algorithm::GP3P));
        ransac.sac_model_ = absposeproblem_ptr;
        ransac.threshold_ = 16;
        ransac.max_iterations_ = 50;
        // initial guess not needed...
        // run the ransac
        if(adapter.getNumberCorrespondences() < 7) {
          ransacLoopClosureTimer.stop();
          continue;
        }
        ransac.computeModel(0);
        attempts++;
        const size_t numInliers = ransac.inliers_.size();
        const double inlierRatio = double(ransac.inliers_.size())/double(adapter.getNumberCorrespondences());
        if(numInliers<10 || inlierRatio < 0.7) {
          ransacLoopClosureTimer.stop();
          continue;
        }
        // remember inliers
        const size_t numCorrespondences = adapter.getNumberCorrespondences();
        std::vector<bool> inliers(numCorrespondences, false);
        for(size_t i=0; i<ransac.inliers_.size(); ++i) {
          inliers.at(size_t(ransac.inliers_.at(i))) = true;
        }
        ransacLoopClosureTimer.stop();

        // 4. refine
        attemptLoopClosureTimer.start();
        const uint64_t frameId = framesInOut->id();
        Eigen::Matrix4d T_Sold_Snew_mat = Eigen::Matrix4d::Identity();
        T_Sold_Snew_mat.topLeftCorner<3,4>() = ransac.model_coefficients_;
        kinematics::Transformation T_Sold_Snew(T_Sold_Snew_mat);

        // set up ceres problem
        ::ceres::Problem::Options problemOptions;
        problemOptions.manifold_ownership =
            ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
        problemOptions.loss_function_ownership =
            ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
        problemOptions.cost_function_ownership =
            ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
        ::ceres::Problem quickSolver(problemOptions);
        ::ceres::CauchyLoss cauchyLoss(3);
        ceres::PoseManifold pose6dParameterisation;
        ceres::HomogeneousPointManifold homogeneousPointParameterisation;

        // parameters: sensor pose and extrinsics
        std::shared_ptr<ceres::PoseParameterBlock> pose(
              new ceres::PoseParameterBlock(T_Sold_Snew, 1));
        quickSolver.AddParameterBlock(pose->parameters(), 7, &pose6dParameterisation);
        std::vector<std::shared_ptr<ceres::PoseParameterBlock>> extrinsics;
        for(size_t i=0; i<framesInOut->numFrames(); ++i) {
          kinematics::Transformation T_SC = estimator.extrinsics(StateId(frameId), uchar(i));
          extrinsics.push_back(std::shared_ptr<ceres::PoseParameterBlock>(
                                 new  ceres::PoseParameterBlock(T_SC, i+2)));
          quickSolver.AddParameterBlock(extrinsics.back()->parameters(), 7,
                                        &pose6dParameterisation);
          quickSolver.SetParameterBlockConstant(extrinsics.back()->parameters());
        }
        const size_t landmarkIdOffset = framesInOut->numFrames()+2;
        std::map<size_t, std::shared_ptr<ceres::ParameterBlock>> lms;
        std::vector<std::shared_ptr<ceres::ReprojectionError2dBase>> reprojectionErrors;
        std::vector<std::pair<const double*, const double*>> extrinsicsAndLandmarks;

        // add error terms and create landmarks if necessary
        for (size_t k = 0; k < numCorrespondences; ++k) {
          if (inliers[k]) {
            // get the landmark id:
            size_t camIdx = size_t(adapter.camIndex(k));
            size_t keypointIdx = size_t(adapter.keypointIndex(k));
            KeypointIdentifier kid(frameId, camIdx, keypointIdx);
            uint64_t lmId = matches.at(kid);
            std::shared_ptr<ceres::ParameterBlock> landmark;
            if(!lms.count(lmId+landmarkIdOffset)) {
              landmark.reset(new ceres::HomogeneousPointParameterBlock(
                               points.at(lmId), lmId+landmarkIdOffset));
              quickSolver.AddParameterBlock(
                    landmark->parameters(), 4, &homogeneousPointParameterisation);
              quickSolver.SetParameterBlockConstant(landmark->parameters());
              lms[lmId+landmarkIdOffset] = landmark;
            } else {
              landmark = lms.at(lmId+landmarkIdOffset);
            }
            Eigen::Vector2d kp;
            if(framesInOut->getKeypoint(camIdx, keypointIdx, kp)) {
              double size = 1.0;
              framesInOut->getKeypointSize(camIdx, keypointIdx, size);
              switch (distortionType) {
                case okvis::cameras::NCameraSystem::RadialTangential: {
                  std::shared_ptr<ceres::ReprojectionError<cameras::PinholeCamera<
                      cameras::RadialTangentialDistortion>>> reprojectionError(
                      new ceres::ReprojectionError<cameras::PinholeCamera<
                        cameras::RadialTangentialDistortion>>(
                        framesInOut->geometryAs<
                          cameras::PinholeCamera<cameras::RadialTangentialDistortion>>(camIdx),
                        camIdx,kp, 64.0 / (size * size) * Eigen::Matrix2d::Identity()));
                  reprojectionErrors.push_back(reprojectionError);
                  quickSolver.AddResidualBlock(
                        reprojectionError.get(), &cauchyLoss, pose->parameters(),
                        landmark->parameters(), extrinsics[camIdx]->parameters());
                  break;
                }
                case okvis::cameras::NCameraSystem::Equidistant: {
                  std::shared_ptr<ceres::ReprojectionError<cameras::PinholeCamera<
                      cameras::EquidistantDistortion>>> reprojectionError(
                      new ceres::ReprojectionError<cameras::PinholeCamera<
                        cameras::EquidistantDistortion>>(
                        framesInOut->geometryAs<cameras::PinholeCamera<
                          cameras::EquidistantDistortion>>(camIdx),
                        camIdx,kp, 64.0 / (size * size) * Eigen::Matrix2d::Identity()));
                  reprojectionErrors.push_back(reprojectionError);
                  quickSolver.AddResidualBlock(
                        reprojectionError.get(), &cauchyLoss, pose->parameters(),
                        landmark->parameters(), extrinsics[camIdx]->parameters());
                  break;
                }
                case okvis::cameras::NCameraSystem::RadialTangential8: {
                  std::shared_ptr<ceres::ReprojectionError<cameras::PinholeCamera<
                      cameras::RadialTangentialDistortion8>>> reprojectionError(
                      new ceres::ReprojectionError<cameras::PinholeCamera<
                        cameras::RadialTangentialDistortion8>>(
                        framesInOut->geometryAs<cameras::PinholeCamera<
                          cameras::RadialTangentialDistortion8>>(camIdx),
                        camIdx,kp, 64.0 / (size * size) * Eigen::Matrix2d::Identity()));
                  reprojectionErrors.push_back(reprojectionError);
                  quickSolver.AddResidualBlock(
                        reprojectionError.get(), &cauchyLoss, pose->parameters(),
                        landmark->parameters(), extrinsics[camIdx]->parameters());
                  break;
                }
                case okvis::cameras::NCameraSystem::NoDistortion: {
                  std::shared_ptr<ceres::ReprojectionError<cameras::EucmCamera>> reprojectionError(
                          new ceres::ReprojectionError<cameras::EucmCamera>(
                                  framesInOut->geometryAs<cameras::EucmCamera>(camIdx),
                                  camIdx,kp, 64.0 / (size * size) * Eigen::Matrix2d::Identity()));
                  reprojectionErrors.push_back(reprojectionError);
                  quickSolver.AddResidualBlock(reprojectionError.get(), &cauchyLoss, pose->parameters(), landmark->parameters(),
                                               extrinsics[camIdx]->parameters());
                  break;
                }
                default:
                  OKVIS_THROW(Exception, "Unsupported distortion type.")
                  break;
              }
              extrinsicsAndLandmarks.push_back(
                    std::pair<const double*, const double*>(
                      landmark->parameters(), extrinsics[camIdx]->parameters()));
            }
          }
        }

        // get solution
        ::ceres::Solver::Options solverOptions;
        ::ceres::Solver::Summary summary;
        solverOptions.num_threads = params.estimator.realtime_num_threads;
        solverOptions.max_num_iterations = params.estimator.realtime_max_iterations;
        solverOptions.minimizer_progress_to_stdout = false;
        ::ceres::Solve(solverOptions, &quickSolver, &summary);
        T_Sold_Snew = pose->estimate();

        // get uncertainty: lhs to be filled on the fly
        Eigen::Matrix<double,6,6> H = Eigen::Matrix<double,6,6>::Zero();
        for(size_t e = 0; e < reprojectionErrors.size(); ++e) {
          // fill lhs Hessian
          const double* pars[3];
          pars[0] = pose->parameters();
          pars[1] = extrinsicsAndLandmarks.at(e).first;
          pars[2] = extrinsicsAndLandmarks.at(e).second;
          const auto & reprojectionError = reprojectionErrors.at(e);
          double* jacobians[3];
          double* jacobiansMinimal[3];
          Eigen::Vector2d err;
          Eigen::Matrix<double, 2, 7> jacobian;
          Eigen::Matrix<double, 2, 6> jacobianMinimal;
          jacobians[0] = jacobian.data();
          jacobiansMinimal[0] = jacobianMinimal.data();
          jacobians[1] = nullptr;
          jacobiansMinimal[1] = nullptr;
          jacobians[2] = nullptr;
          jacobiansMinimal[2] = nullptr;
          reprojectionError->EvaluateWithMinimalJacobians(
                pars, err.data(), jacobians, jacobiansMinimal);
          H += jacobianMinimal.transpose()*jacobianMinimal;
        }

        // 5. enforce relative transformation
        const size_t numIter = 20; // high max. but uses low relative tolerance.
        bool skipFullGraphOptimisation = false;
        bool loopClosureAttemptSuccessful =
            estimator.attemptLoopClosure(
              StateId(oldFrame->id()), StateId(frameId), T_Sold_Snew, H,
              skipFullGraphOptimisation,
              numIter, params.estimator.realtime_num_threads);
        if(!loopClosureAttemptSuccessful) {
          LOG(INFO) << "unsuccessful loop closure frame "<< poseId;
          attemptLoopClosureTimer.stop();
          continue;
        }
        attemptLoopClosureTimer.stop();

        // 6. bring back old landmarks
        TimerSwitchable addLoopClosureTimer("2.7 add loop closure");
        std::set<LandmarkId> loopClosureLandmarks;
        estimator.addLoopClosureFrame(StateId(oldFrame->id()), loopClosureLandmarks,
                                      skipFullGraphOptimisation);
        addLoopClosureTimer.stop();

        // 7. properly match against all loopClosure frame points
        TimerSwitchable matchLoopClosureTimer("2.8 match loop closure");
        int loopClosureMatches = 0;

        switch (distortionType) {
          case okvis::cameras::NCameraSystem::RadialTangential: {
            loopClosureMatches =
                matchToMap<cameras::PinholeCamera<cameras::RadialTangentialDistortion>>(
                    estimator, params, framesInOut->id(), &loopClosureLandmarks);
            break;
          }
          case okvis::cameras::NCameraSystem::Equidistant: {
            loopClosureMatches = matchToMap<cameras::PinholeCamera<cameras::EquidistantDistortion>>(
                estimator, params, framesInOut->id(), &loopClosureLandmarks);
            break;
          }
          case okvis::cameras::NCameraSystem::RadialTangential8: {
            loopClosureMatches =
                matchToMap<cameras::PinholeCamera<cameras::RadialTangentialDistortion8>>(
                    estimator, params, framesInOut->id(), &loopClosureLandmarks);
            break;
          }
          case okvis::cameras::NCameraSystem::NoDistortion: {
            loopClosureMatches = matchToMap<cameras::EucmCamera>(
                    estimator, params, framesInOut->id(), &loopClosureLandmarks);
            break;
          }
          default:
            OKVIS_THROW(Exception, "Unsupported distortion type.")
            break;
        }

        //LOG(INFO) << "loop closure matches: " << loopClosureMatches << " vs " << inliers.size() <<
        //             " no. lc lm:" << loopClosureLandmarks.size();
        LOG(INFO) << "LOOP CLOSURE: current frame " << frameId
                  << ", matching to keyframe " << oldFrame->id() << ", "
                  << loopClosureMatches << " matches";

        matchLoopClosureTimer.stop();

        // 10. re-decide keyframe:
        *asKeyframe = doWeNeedANewKeyframe(estimator, framesInOut);

        break; // only consider oldest keyframe match.
      }
    }
    if(matchLoopClosureTimer.isTiming()) {
      matchLoopClosureTimer.stop();
    }

    // if keyframe, we add to relocalisation database
    if(*asKeyframe) {
      dBow_->database.add(features);
      dBow_->poseIds.push_back(framesInOut->id());
    }
  }

#ifdef OKVIS_USE_NN
  // This needs to be after keyframe re-decision, otherwise we might delete a frame before the CNN
  // finishes. This could obviously be done in a smarter way though.
  if(params.frontend.use_cnn && isInitialized_) {
    if(*asKeyframe) {
      // clean up threads if still running
      std::set<StateId> toDelete;
      for(auto threads = cnnThreads_.begin(); threads != cnnThreads_.end(); ++threads) {
        bool deleting = true;
        for(size_t i=0; i<threads->second.size(); ++i) {
          if(threads->second[i] && estimator.multiFrame(StateId(threads->first))->isClassified(i)) {
            threads->second[i]->join();
            delete threads->second[i];
            threads->second[i] = nullptr;
          } else {
            deleting = false;
          }
        }
        if(deleting) {
          toDelete.insert(StateId(threads->first));
        }
      }
      for(const auto & deleting : toDelete) {
        cnnThreads_.erase(deleting);
      }

      // launch classification in background
      cnnThreads_[StateId(framesInOut->id())] = std::vector<std::thread*>(
            params.nCameraSystem.numCameras(), nullptr);
      for (size_t i = 0; i < params.nCameraSystem.numCameras(); ++i) {
        cnnThreads_[StateId(framesInOut->id())].at(i) =
            new std::thread(&MultiFrame::computeClassifications,framesInOut.get(), i,
                      64*(framesInOut->image(i).cols/64), 64*(framesInOut->image(i).rows/64));
      }
    }
  }
#else
  OKVIS_ASSERT_TRUE(Exception, !params.frontend.use_cnn,
                    "Requested CNN classification, but not compiled with USE_NN option.")
#endif

  // do stereo match -- get new landmarks only when this is a keyframe
  if(*asKeyframe) {
    TimerSwitchable matchStereoTimer("2.9 match stereo");
    switch (distortionType) {
      case okvis::cameras::NCameraSystem::RadialTangential: {
        matchStereo<cameras::PinholeCamera<cameras::RadialTangentialDistortion>>(
              estimator, framesInOut, params, *asKeyframe);
        break;
      }
      case okvis::cameras::NCameraSystem::Equidistant: {
        matchStereo<cameras::PinholeCamera<cameras::EquidistantDistortion>>(
              estimator, framesInOut, params, *asKeyframe);
        break;
      }
      case okvis::cameras::NCameraSystem::RadialTangential8: {
        matchStereo<okvis::cameras::PinholeCamera<cameras::RadialTangentialDistortion8>>(
              estimator, framesInOut, params, *asKeyframe);
        break;
      }
      case okvis::cameras::NCameraSystem::NoDistortion: {
        matchStereo<okvis::cameras::EucmCamera>(
                estimator, framesInOut, params, *asKeyframe);
        break;
        }
      default:
        OKVIS_THROW(Exception, "Unsupported distortion type.")
        break;
    }
    matchStereoTimer.stop();
  }

#ifdef OKVIS_USE_NN
  if(params.frontend.use_cnn) {
    // remove matches into dynamic areas
    // get all landmarks
    MapPoints pointMap;
    estimator.getLandmarks(pointMap);
    int removeCtr = 0;
    for(MapPoints::iterator it = pointMap.begin(); it != pointMap.end(); ++it) {
      bool remove = false;
      if(it->second.classification == 10 || it->second.classification == 11) {
        remove = true;
      } else {
      for(auto& obs : it->second.observations) {
        if(!estimator.isKeyframe(StateId(obs.frameId))) continue;
        auto frame = estimator.multiFrame(StateId(obs.frameId));
        if(!frame->isClassified(obs.cameraIndex)) continue;
        cv::Mat classification;
        if(frame->getClassification(obs.cameraIndex, obs.keypointIndex, classification)) {
          if(classification.at<float>(10) > 3.5f) { // Sky
            remove = true;
            removeCtr++;
            Eigen::Vector2d kpt;
            frame->getKeypoint(obs.cameraIndex, obs.keypointIndex, kpt);
            estimator.setLandmarkClassification(it->first, 10);
            break;
          }
          if(classification.at<float>(11) > 53.5f) { // Person
            remove = true;
            removeCtr++;
            estimator.setLandmarkClassification(it->first, 11);
            break;
          }
        }
      }
      }
      if(remove) {
        std::set<KeypointIdentifier> observations = it->second.observations;
        for(auto& obs : observations) {
          estimator.setObservationInformation(
                StateId(obs.frameId), obs.cameraIndex, obs.keypointIndex,
                Eigen::Matrix2d::Identity()*0.0001);
        }
      }
    }
  }
#endif
  estimator.cleanUnobservedLandmarks();

  return trackingQuality >= 0.01;
}

// Propagates pose, speeds and biases with given IMU measurements.
bool Frontend::propagation(
    const okvis::ImuMeasurementDeque& imuMeasurements, const okvis::ImuParameters& imuParams,
    okvis::kinematics::Transformation& T_WS_propagated, okvis::SpeedAndBias& speedAndBiases,
    const okvis::Time& t_start, const okvis::Time& t_end, Eigen::Matrix<double, 15, 15>* covariance,
    Eigen::Matrix<double, 15, 15>* jacobian) const {

  if (imuMeasurements.size() < 2) {
    LOG(WARNING) << "- Skipping propagation as only one IMU measurement has been given to frontend."
                 << " Normal when starting up.";
    return 0;
  }
  int measurements_propagated =
      okvis::ceres::ImuError::propagation(imuMeasurements, imuParams, T_WS_propagated,
                                          speedAndBiases, t_start, t_end, covariance, jacobian);

  return measurements_propagated > 0;
}

void Frontend::endCnnThreads() {
  for(auto & threads : cnnThreads_) {
    for(auto & thread : threads.second) {
      if(thread) {
        thread->join();
        delete thread;
        thread = nullptr;
      }
    }
  }
}

void Frontend::clear()
{
  endCnnThreads();
  isInitialized_ = false;        // Is the pose initialised?
  dBow_->database.clear();
  dBow_->poseIds.clear(); // Store the multiframe IDs corresponsind to the dBow ones
  trackingLost_ = false; // Is the tracking currently lost?
}

// Decision whether a new frame should be keyframe or not.
bool Frontend::doWeNeedANewKeyframe(const Estimator &estimator,
                                    std::shared_ptr<okvis::MultiFrame> currentFrame) {
  if (estimator.numFrames() < 4) {
    // just starting, so yes, we need this as a new keyframe
    return true;
  }

  if (!isInitialized_) return false;

  int intersectionCount = 0;
  int unionCount = 0;

  size_t numKeypoints = 0;
  size_t numMatches = 0;

  // go through all the frames and try to match the initialized keypoints
  std::set<uint64_t> lmIds;
  for (size_t im = 0; im < currentFrame->numFrames(); ++im) {
    const int rows = currentFrame->image(im).rows/10;
    const int cols = currentFrame->image(im).cols/10;

    cv::Mat matches = cv::Mat::zeros(rows, cols, CV_8UC1);
    cv::Mat detections = cv::Mat::zeros(rows, cols, CV_8UC1);

    const size_t numB = currentFrame->numKeypoints(im);
    numKeypoints += numB;
    const double radius = double(std::min(rows,cols))*kptrad;
    cv::KeyPoint keypoint;
    for (size_t k = 0; k < numB; ++k) {
      currentFrame->getCvKeypoint(im, k, keypoint);
      cv::circle(detections, keypoint.pt*0.1, int(radius), cv::Scalar(255), cv::FILLED);
      uint64_t lmId = currentFrame->landmarkId(im, k);
      if (lmId != 0) {
        cv::circle(matches, keypoint.pt*0.1, int(radius), cv::Scalar(255), cv::FILLED);
        numMatches++;
        lmIds.insert(lmId);
      }
    }

    // IoU
    cv::Mat intersectionMask, unionMask;
    cv::bitwise_and(matches, detections, intersectionMask);
    cv::bitwise_or(matches, detections, unionMask);
    intersectionCount += cv::countNonZero(intersectionMask);
    unionCount += cv::countNonZero(unionMask);
  }

  double overlap = double(intersectionCount)/double(unionCount);

  std::set<StateId> allFrames = estimator.keyFrames();
  allFrames.insert(estimator.loopClosureFrames().begin(), estimator.loopClosureFrames().end());
  for(size_t age = 0; age < estimator.numFrames(); ++age) {
    auto id = estimator.stateIdByAge(age);
    if(!estimator.isInImuWindow(id)) {
      break;
    }
    if(estimator.isKeyframe(id)) {
      allFrames.insert(id);
    }
  }
  double overlapOthers = 0.0;
  for(auto frame : allFrames) {
    int intersectionCount = 0;
    int unionCount = 0;

    //size_t numKeypoints = 0;
    size_t numMatches = 0;

    // go through all the frames and try to match the initialized keypoints
    auto otherFrame = estimator.multiFrame(frame);
    for (size_t im = 0; im < otherFrame->numFrames(); ++im) {
      const int rows = otherFrame->image(im).rows/10;
      const int cols = otherFrame->image(im).cols/10;

      cv::Mat matches = cv::Mat::zeros(rows, cols, CV_8UC1);
      cv::Mat detections = cv::Mat::zeros(rows, cols, CV_8UC1);

      const size_t numB = otherFrame->numKeypoints(im);

      const double radius = double(std::min(rows,cols))*kptrad;
      cv::KeyPoint keypoint;
      for (size_t k = 0; k < numB; ++k) {
        otherFrame->getCvKeypoint(im, k, keypoint);
        cv::circle(detections, keypoint.pt*0.1, int(radius), cv::Scalar(255), cv::FILLED);
        uint64_t lmId = otherFrame->landmarkId(im, k);
        if (lmId != 0 && lmIds.count(lmId)) {
          cv::circle(matches, keypoint.pt*0.1, int(radius), cv::Scalar(255), cv::FILLED);
          numMatches++;
        }
      }

      // IoU
      cv::Mat intersectionMask, unionMask;
      cv::bitwise_and(matches, detections, intersectionMask);
      cv::bitwise_or(matches, detections, unionMask);
      intersectionCount += cv::countNonZero(intersectionMask);
      unionCount += cv::countNonZero(unionMask);
    }

    overlapOthers = std::max(overlapOthers, double(intersectionCount)/double(unionCount));
  }

  overlap = std::min(overlapOthers, overlap);

  // take a decision
  if(numKeypoints < 7 * currentFrame->numFrames()) {
    // a respectable keyframe needs some detections...
    return false;
  }
  if (float(overlap) > keyframeInsertionOverlapThreshold_
      /*&& double(numMatches)/double(numKeypoints) > 0.35*/) {
    return false;
  } else {
    return true;
  }
}

// Match a new multiframe to existing keyframes
template <class CAMERA_GEOMETRY>
int Frontend::matchToMap(Estimator &estimator, const okvis::ViParameters& params,
                         const uint64_t currentFrameId,
                         const std::set<LandmarkId>* loopClosureLandmarksToUseExclusively) {

  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return 0;
  }

  // get all landmarks
  MapPoints pointMap;
  estimator.getLandmarks(pointMap);

  // these may be needed for loop-closure map fusion
  std::vector<LandmarkId> oldIds, newIds;

  // match
  int ctr = 0;
  const kinematics::Transformation T_WS1 = estimator.pose(StateId(currentFrameId));
  for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
    // the current frame to match
    const MultiFramePtr multiFrame = estimator.multiFrame(StateId(currentFrameId));
    const size_t numKeypoints = multiFrame->numKeypoints(im);
    if(numKeypoints == 0) {
      continue; // no points -- bad!
    }

    const size_t num_matching_threads = size_t(params.frontend.num_matching_threads);

    std::vector<double> distances(numKeypoints,briskMatchingThreshold_);
    std::vector<LandmarkId> lmIds(numKeypoints);
    AlignedVector<Eigen::Vector4d> hps_W(numKeypoints, Eigen::Vector4d::Zero());
    std::vector<size_t> ctrs(num_matching_threads);

    // multithreaded matching
    std::vector<std::thread*> threads(num_matching_threads, nullptr);
    for(size_t t = 0; t<num_matching_threads; ++t) {
      threads[t] = new std::thread(
            &Frontend::matchToMapByThread<CAMERA_GEOMETRY>, this, t, num_matching_threads,
            std::cref(estimator), std::cref(params),
            currentFrameId, loopClosureLandmarksToUseExclusively, std::cref(T_WS1), numKeypoints,
            std::cref(pointMap), im, std::cref(multiFrame), std::ref(distances),
            std::ref(lmIds), std::ref(hps_W), std::ref(ctrs));
    }

    for(size_t t = 0; t<num_matching_threads; ++t) {
      threads[t]->join();
      delete threads[t];
      ctr += ctrs[t];
    }

    // now insert observations
    for(size_t k = 0; k < numKeypoints; ++k) {
      uint64_t previousId = multiFrame->landmarkId(im,k);
      if(lmIds[k].isInitialised()) {

        if(previousId && loopClosureLandmarksToUseExclusively) {
          // remove
          estimator.removeObservation(StateId(currentFrameId), im, k);
          oldIds.push_back(LandmarkId(previousId));
          newIds.push_back(lmIds[k]);
        }

        if(hps_W[k].norm()>1.0e-22 && !estimator.isLandmarkInitialised(lmIds[k])) { //ugly
          estimator.setLandmark(lmIds[k], hps_W[k], true);
        }

        multiFrame->setLandmarkId(im, k, lmIds[k].value());
        estimator.addObservation<CAMERA_GEOMETRY>(
              lmIds[k], StateId(currentFrameId), im, k);
        ctr++;
      }
    }
  }

  // merge landmarks, if loop-closure matching
  if(loopClosureLandmarksToUseExclusively) {
    estimator.mergeLandmarks(oldIds, newIds);
  }

  // remove outliers
  static const bool removeOutliers = true;
  if (isInitialized_) {
    runRansac3d2d(estimator, params.nCameraSystem, estimator.multiFrame(StateId(currentFrameId)),
                  removeOutliers);
  }

  return ctr;
}

// Match a new multiframe to existing keyframes:
template <class CAMERA_GEOMETRY>
void Frontend::matchToMapByThread(
    size_t threadIdx, size_t numThreads, const Estimator &estimator,
    const okvis::ViParameters& params, const uint64_t currentFrameId,
    const std::set<LandmarkId>* loopClosureLandmarksToUseExclusively,
    const kinematics::Transformation& T_WS1, size_t numKeypoints, const MapPoints& pointMap,
    size_t im, const MultiFramePtr&  multiFrame, std::vector<double>& distances,
    std::vector<LandmarkId>& lmIds, AlignedVector<Eigen::Vector4d>& hps_W,
    std::vector<size_t>& ctrs) const {

  const kinematics::Transformation T_SC = *params.nCameraSystem.T_SC(im);
  const kinematics::Transformation T_WC1 = T_WS1 * T_SC;
  const kinematics::Transformation T_CW1 = T_WC1.inverse();

  ctrs[threadIdx] = 0;

  // go through all landmarks
  for(MapPoints::const_iterator it = pointMap.begin(); it != pointMap.end(); ++it) {
    if(loopClosureLandmarksToUseExclusively) {
      if(!loopClosureLandmarksToUseExclusively->count(it->first)) {
        continue; // skip non-loop-closure points in this case
      }
    }

    // FoV check
    const Eigen::Vector4d hp_W = it->second.point;
    Eigen::Vector2d kp;
    if(cameras::ProjectionStatus::Successful !=
      multiFrame->geometryAs<CAMERA_GEOMETRY>(im)->projectHomogeneous(T_CW1*hp_W, &kp)) {
      continue;
    }

    // obtain map point descriptor
    cv::Mat mapPointDescriptors(int(it->second.observations.size()),48, CV_8UC1);
    const uint64_t landmarkId = it->first.value();
    int o=0;
    for(auto obsiter = it->second.observations.rbegin(); obsiter !=
        it->second.observations.rend(); ++obsiter) {
      const KeypointIdentifier kid = *obsiter;
      std::memcpy(
            mapPointDescriptors.data+48*o,
            estimator.multiFrame(StateId(kid.frameId))->keypointDescriptor(
              kid.cameraIndex, kid.keypointIndex), 48);
      o++;
    }

    if(mapPointDescriptors.rows==0) {
      // no observations -- weird.
      continue;
    }

    // match all present descriptors
    const size_t segment = numKeypoints/numThreads;
    const size_t startK = segment*threadIdx;
    const size_t endK = threadIdx+1 == numThreads ? numKeypoints : startK + segment;
    for(size_t k = startK; k < endK; k++) {

      uint64_t previousId = multiFrame->landmarkId(im,k);
      if(previousId&&!loopClosureLandmarksToUseExclusively) {
        continue; // already matched
      }

      // also check image distance, unless tracking lost.
      Eigen::Vector2d keypoint;
      multiFrame->getKeypoint(im, k, keypoint);
      if ((kp - keypoint).norm() > 10.0) {
        continue;
      }

      int d=0;
      for(auto obsiter = it->second.observations.rbegin(); obsiter !=
          it->second.observations.rend(); ++obsiter) {
        const auto dist = brisk::Hamming::PopcntofXORed(
            multiFrame->keypointDescriptor(im, k), mapPointDescriptors.data + d*48, 3);
        d++;
        if(dist < distances[k]) {
          if(landmarkId==previousId) {
            ctrs[threadIdx]++; // still counts, already correct match...
            break; // the match is already done...
          }

          if(dist<distances[k]) {
            distances[k] = dist;
            lmIds[k] = LandmarkId(landmarkId);
          }
        }
      }
    }
  }
}

/// \brief Temporary match info storage.
struct MatchInfo {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d hp_W; ///< 3D point.
  size_t k1 = 0; ///< Match idx.
  bool matching = false; ///< Does it match?
  bool initialisable = false; ///< Initialisable?
  double quality = 0.0; ///< 3D quality.
};

template <class CAMERA_GEOMETRY>
int Frontend::matchMotionStereo(Estimator& estimator, const ViParameters &params,
                                 const uint64_t currentFrameId, bool& rotationOnly) {
  int retCtr = 0;
  rotationOnly = true;

  kinematics::Transformation T_WS1 = estimator.pose(StateId(currentFrameId));

  // find close frames
  TimerSwitchable matchMotionStereoTimer2("2.2.1 match motion stereo: prepare");
  std::set<StateId> allFrames;
  allFrames.insert(estimator.keyFrames().begin(), estimator.keyFrames().end());
  allFrames.insert(estimator.imuFrames().begin(), estimator.imuFrames().end());
  for(const auto & id : estimator.imuFrames()) {
    if(!estimator.isKeyframe(id)) {
      allFrames.erase(id);
    }
  }
  StateId previousFrameId = estimator.stateIdByAge(1);
  std::vector<std::pair<double, StateId>> overlaps;
  for(auto & id : allFrames) {
    if(id == previousFrameId) {
      overlaps.push_back(std::pair<double, StateId>(1.0, id));
      continue;
    }
    const double overlap = estimator.overlapFraction(
          estimator.multiFrame(previousFrameId),estimator.multiFrame(id));
    overlaps.push_back(std::pair<double, StateId>(overlap, id));
  }
  std::sort(overlaps.begin(), overlaps.end());
  std::vector<StateId> matchFrameIds;

  for(size_t i=0; i<overlaps.size(); ++i) {
    if(overlaps[overlaps.size()-i-1].first <= 1.0e-8) break;
    matchFrameIds.push_back(overlaps[overlaps.size()-i-1].second);
  }

  matchMotionStereoTimer2.stop();

  kinematics::Transformation T_WS0;
  bool firstFrame = true;
  for (auto olderFrameId : matchFrameIds) {
    T_WS0 = estimator.pose(olderFrameId);
    for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
      const kinematics::Transformation T_SC = *params.nCameraSystem.T_SC(im);
      const kinematics::Transformation T_WC0 = T_WS0 * T_SC;
      const kinematics::Transformation T_WC1 = T_WS1 * T_SC;
      // match
      MultiFramePtr multiFrame0 = estimator.multiFrame(olderFrameId);
      MultiFramePtr multiFrame1 = estimator.multiFrame(StateId(currentFrameId));
      const size_t k0Size = multiFrame0->numKeypoints(im);
      const size_t k1Size = multiFrame1->numKeypoints(im);
      const auto camera = multiFrame0->geometryAs<CAMERA_GEOMETRY>(im);
      const double f0 = 0.5* (camera->focalLengthU() + camera->focalLengthV());

      AlignedVector<MatchInfo> matchInfos(k0Size);

      // vector container stores threads
      std::vector<std::thread> workers;
      for (size_t t = 0; t < size_t(params.frontend.num_matching_threads); t++) {
        workers.push_back(std::thread([this, t, k0Size, im, &multiFrame0, &estimator, f0, k1Size,
                                      &T_WC0, &T_WC1, &multiFrame1, &olderFrameId, &matchInfos,
                                      &params, &camera]() {
          for(size_t k0 = t; k0 < k0Size; k0 += size_t(params.frontend.num_matching_threads)) {
            uint64_t id0 = multiFrame0->landmarkId(im, k0);
            if(id0) {
              if(!estimator.isLandmarkAdded(LandmarkId(id0))) {
                continue; // weird
              }
              if(estimator.isLandmarkInitialised(LandmarkId(id0))) {
                continue; // already matched
              }
            }

            double distances = briskMatchingThreshold_;
            bool initialisable = false;
            double quality = 0.0;
            Eigen::Vector4d hps_W;
            size_t k1_max=1000;

            // pre-fetch frame 0 stuff
            const uchar* d0 = multiFrame0->keypointDescriptor(im, k0);
            double size0;
            multiFrame0->getKeypointSize(im, k0, size0);
            Eigen::Vector2d pt0;
            multiFrame0->getKeypoint(im, k0, pt0);
            Eigen::Vector3d e0_C;
            if(!multiFrame0->getBackProjection(im, k0, e0_C)) continue;
            const Eigen::Vector3d e0_W = (T_WC0.C()*e0_C).normalized();
            const double sigma = size0/f0 * 0.125;

            for(size_t k1 = 0; k1 < k1Size; ++k1) {
              const uint64_t id1 = multiFrame1->landmarkId(im, k1);
              if(id1) {
                continue; // already matched
              }
              if(estimator.isObserved(KeypointIdentifier{olderFrameId.value(), im, k0})) {
                continue; // already matched
              }
              const auto dist = brisk::Hamming::PopcntofXORed(
                  d0, multiFrame1->keypointDescriptor(im, k1), 3);
              if(dist < distances) {
                // it's a match!

                // triangulate
                bool isValid = false;
                bool isParallel = false;
                Eigen::Vector3d e1_C;
                if(!multiFrame1->getBackProjection(im, k1, e1_C)) continue;
                const Eigen::Vector3d e1_W = (T_WC1.C()*e1_C).normalized();
                if(e0_W.dot(e1_W) < 0.5) continue;

                Eigen::Vector4d hp_W = triangulation::triangulateFast(
                      T_WC0.r(), e0_W, T_WC1.r(), e1_W, sigma, isValid, isParallel);

                if(!isValid) {
                  continue;
                }

                // check if too close (out of focus)
                const Eigen::Vector4d hp_C0 = (T_WC0.inverse()*hp_W);
                const Eigen::Vector4d hp_C1 = (T_WC1.inverse()*hp_W);

                if(e0_W.transpose()*e1_W < 0.8) {
                  isValid = false;
                }

                if(!isParallel) {
                  hp_W = hp_W/hp_W[3];
                  if(hp_C0[2]/hp_C0[3] < 0.2) {
                    isValid = false;
                  }
                  if(hp_C1[2]/hp_C1[3] < 0.2) {
                    isValid = false;
                  }
                }

                // remember
                if(/*dist<distances && */isValid) {
                  k1_max = k1;
                  distances=dist;
                  quality = acos((hp_W.head<3>()-T_WC0.r()).normalized()
                                 .dot((hp_W.head<3>()-T_WC1.r()).normalized()));
                  hps_W = hp_W;
                  initialisable = !isParallel;
                }
              }
            }

            // add observations and initialise
            if(distances < briskMatchingThreshold_) {
              Eigen::Vector2d pt1p;
              Eigen::Vector2d pt1;
              multiFrame1->getKeypoint(im, k1_max, pt1);
              auto s1 = camera->projectHomogeneous(T_WC1.inverse()*hps_W, &pt1p);
              if(s1 == cameras::ProjectionStatus::Successful && (pt1-pt1p).norm()<4.0) {
                matchInfos[k0] = MatchInfo{hps_W, k1_max, true, initialisable, quality};
              }
            }
          }
        }));
      }

      // join all matcher threads
      std::for_each(workers.begin(), workers.end(), [](std::thread &worker) {
          worker.join();
      });

      // finally insert the actual matches
      for(size_t k0=0; k0<k0Size; ++k0) {
        const MatchInfo & mInfo = matchInfos.at(k0);
        uint64_t id0 = multiFrame0->landmarkId(im, k0);
        if(!mInfo.matching) {
          continue;
        }

        if(id0) {
          if(estimator.isLandmarkInitialised(LandmarkId(id0))) {
            continue; // already matched
          }
          if(!estimator.isLandmarkAdded(LandmarkId(id0))) {
            continue; // weird!
          }
        }
        if(estimator.isObserved(KeypointIdentifier{olderFrameId.value(), im, k0})) {
          continue; // already matched
        }

        const uint64_t id1 = multiFrame1->landmarkId(im, mInfo.k1);
        if(id1) {
          continue; // already matched
        }

        if(id0){
          MapPoint2 lm;
          estimator.getLandmark(LandmarkId(id0), lm);
          if(lm.quality<mInfo.quality) {
            estimator.setLandmark(LandmarkId(id0), mInfo.hp_W, mInfo.initialisable);
          }
        } else {
          id0 = estimator.addLandmark(mInfo.hp_W, mInfo.initialisable).value();
          multiFrame0->setLandmarkId(im, k0, id0);
          OKVIS_ASSERT_TRUE_DBG(Exception, estimator.isLandmarkAdded(LandmarkId(id0)),
                              id0<<" not added, bug")
          estimator.addObservation<CAMERA_GEOMETRY>(LandmarkId(id0), StateId(olderFrameId), im, k0);
        }

        multiFrame1->setLandmarkId(im, mInfo.k1, id0);
        estimator.addObservation<CAMERA_GEOMETRY>(
              LandmarkId(id0), StateId(currentFrameId), im, mInfo.k1);
        retCtr++;
      }
    }

    bool rotationOnly_tmp = false;
    static const bool removeOutliers = true;

    // do RANSAC 2D2D for initialization only
    const bool initialisePose =  (!isInitialized_);
    if(!isInitialized_) {
      runRansac2d2d(estimator, params, currentFrameId, olderFrameId.value(), initialisePose,
                    removeOutliers, rotationOnly_tmp);
    }

    if (firstFrame) {
      rotationOnly = rotationOnly_tmp;
      firstFrame = false;
    }
  }

  return retCtr;
}

// Match the frames inside the multiframe to each other to initialise new landmarks.
template <class CAMERA_GEOMETRY>
void Frontend::matchStereo(Estimator &estimator, std::shared_ptr<okvis::MultiFrame> multiFrame,
                           const okvis::ViParameters& params, bool asKeyframe) {
  const size_t camNumber = multiFrame->numFrames();
  const uint64_t mfId = multiFrame->id();

  int ctr = 0;

  // needed later:
  kinematics::Transformation T_WS = estimator.pose(StateId(mfId));

  for (size_t im0 = 0; im0 < camNumber; im0++) {
    const kinematics::Transformation T_SC0 = *params.nCameraSystem.T_SC(im0);

    for (size_t im1 = im0 + 1; im1 < camNumber; im1++) {
      // first, check the possibility for overlap
      // FIXME: implement this in the Multiframe...!!

      // check overlap
      if (!multiFrame->hasOverlap(im0, im1)) {
        continue;
      }

      // useful later:
      const kinematics::Transformation T_SC1 = *params.nCameraSystem.T_SC(im1);
      const kinematics::Transformation T_WC0 = T_WS * T_SC0;
      const kinematics::Transformation T_WC1 = T_WS * T_SC1;

      {
        // match
        MultiFramePtr multiFrame = estimator.multiFrame(StateId(mfId));
        const size_t k0Size = multiFrame->numKeypoints(im0);
        const size_t k1Size = multiFrame->numKeypoints(im1);
        const auto camera0 = multiFrame->geometryAs<CAMERA_GEOMETRY>(im0);
        const auto camera1 = multiFrame->geometryAs<CAMERA_GEOMETRY>(im1);
        const double f0 = 0.5* (camera0->focalLengthU() + camera0->focalLengthV());
        const double f1 = 0.5* (camera1->focalLengthU() + camera1->focalLengthV());
        for(size_t k0 = 0; k0 < k0Size; ++k0) {

          double distances = briskMatchingThreshold_;
          bool initialisable=  false;
          Eigen::Vector4d hps_W;
          size_t k1_match = 0;

          for(size_t k1 = 0; k1 < k1Size; ++k1) {
            const auto dist = brisk::Hamming::PopcntofXORed(
                multiFrame->keypointDescriptor(im0, k0),
                  multiFrame->keypointDescriptor(im1, k1), 3);
            if(dist < distances) {
              // it's a match!
              double size0, size1;
              multiFrame->getKeypointSize(im0, k0, size0);
              multiFrame->getKeypointSize(im1, k1, size1);
              Eigen::Vector2d pt0, pt1;
              multiFrame->getKeypoint(im0, k0, pt0);
              multiFrame->getKeypoint(im1, k1, pt1);
              const double sigma = std::max(size0/f0, size1/f1) * 0.125;

              // triangulate
              bool isValid = false;
              bool isParallel = false;
              Eigen::Vector3d e0_C, e1_C;
              if(!multiFrame->getBackProjection(im0, k0, e0_C)) continue;
              if(!multiFrame->getBackProjection(im1, k1, e1_C)) continue;
              Eigen::Vector3d e0_W = (T_WC0.C()*e0_C).normalized();
              Eigen::Vector3d e1_W = (T_WC1.C()*e1_C).normalized();
              Eigen::Vector4d hp_W = triangulation::triangulateFast(
                    T_WC0.r(), e0_W, T_WC1.r(), e1_W, sigma, isValid, isParallel);

              // check if too close
              const Eigen::Vector4d hp_C0 = (T_WC0.inverse()*hp_W);
              const Eigen::Vector4d hp_C1 = (T_WC1.inverse()*hp_W);
              if(!isParallel) {
                hp_W = hp_W/hp_W[3];

                if(hp_C0[2]/hp_C0[3] < 0.05) {
                  isValid = false;
                }
                if(hp_C1[2]/hp_C1[3] <  0.05) {
                  isValid = false;
                }

                if(e0_W.transpose()*e1_W < 0.8) {
                  isValid = false;
                }
              }

              // add observations and initialise
              if(isValid) {
                distances = dist;
                hps_W = hp_W;
                k1_match = k1;
                initialisable = !isParallel;
              }
            }
          }

          if(distances<briskMatchingThreshold_) {
            Eigen::Vector2d pt0, pt1;
            multiFrame->getKeypoint(im0, k0, pt0);
            multiFrame->getKeypoint(im1, k1_match, pt1);
            uint64_t lmId = 0;
            const uint64_t id0 = multiFrame->landmarkId(im0, k0); // may change!!
            const uint64_t id1 = multiFrame->landmarkId(im1, k1_match);
            bool add0 = false;
            bool add1 = false;
            if(id0 && id1) {
              if(!estimator.isLandmarkInitialised(LandmarkId(id0))) {
                // only re-assess initialisation
                if(initialisable) {
                  //estimator.setLandmarkInitialized(id0, initialiseable);
                  estimator.setLandmark(LandmarkId(id0), hps_W, true); /// \todo check true
                }
              } // else we do nothing, because already initialised and matched.
            } else if(id1) {
              // only add observation into frame0
              lmId = id1;
              add0 = true;

            } else if(id0) {
              // only add observation into frame1
              lmId = id0;
              add1 = true;
            } else {
              if(!asKeyframe){
                continue; // we don't want to create new stuff from non-keyframes
              }
              add0 = true;
              add1 = true;
              // need new point

              lmId = estimator.addLandmark(hps_W, initialisable).value();
              OKVIS_ASSERT_TRUE_DBG(Exception, estimator.isLandmarkAdded(LandmarkId(lmId)),
                                lmId<<" not added, bug")
              ctr++;
              }
            if(add0) {
              // verify (again, because landmark may not have been reset)
              Eigen::Vector2d pt0p;
              MapPoint2 mapPoint;
              estimator.getLandmark(LandmarkId(lmId), mapPoint);
              Eigen::Vector4d hp_eff_W = mapPoint.point;
              auto s0 = camera0->projectHomogeneous(T_WC0.inverse()*hp_eff_W, &pt0p);
              if(s0 == cameras::ProjectionStatus::Successful && (pt0-pt0p).norm()<4.0) {
                // safe to add.
                multiFrame->setLandmarkId(im0, k0, lmId);
                estimator.addObservation<CAMERA_GEOMETRY>(LandmarkId(lmId), StateId(mfId), im0, k0);
              }
            }
            if(add1) {
              // verify (again, because landmark may not have been reset)
              Eigen::Vector2d pt1p;
              MapPoint2 mapPoint;
              estimator.getLandmark(LandmarkId(lmId), mapPoint);
              Eigen::Vector4d hp_eff_W = mapPoint.point;
              auto s1 = camera1->projectHomogeneous(T_WC1.inverse()*hp_eff_W, &pt1p);
              if(s1 == cameras::ProjectionStatus::Successful && (pt1-pt1p).norm()<4.0) {
                multiFrame->setLandmarkId(im1, k1_match, lmId);
                estimator.addObservation<CAMERA_GEOMETRY>(LandmarkId(lmId), StateId(mfId), im1, k1_match);
              }
            }
          }
        }
      }
    }
  }

  // TODO: for more than 2 cameras check that there were no duplications!

  // TODO: ensure 1-1 matching.
}

// Perform 3D/2D RANSAC.
int Frontend::runRansac3d2d(
    Estimator &estimator, const okvis::cameras::NCameraSystem& nCameraSystem,
    std::shared_ptr<okvis::MultiFrame> currentFrame, bool removeOutliers) {
  if (estimator.numFrames() < 2) {
    // nothing to match against, we are just starting up.
    return 1;
  }

  /////////////////////
  //   KNEIP RANSAC
  /////////////////////
  int numInliers = 0;

  // absolute pose adapter for Kneip toolchain
  opengv::absolute_pose::FrameNoncentralAbsoluteAdapter adapter(estimator, nCameraSystem, currentFrame);

  size_t numCorrespondences = adapter.getNumberCorrespondences();
  if (numCorrespondences < 10) return int(numCorrespondences);

  // create a RelativePoseSac problem and RANSAC
  typedef opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem<
        opengv::absolute_pose::FrameNoncentralAbsoluteAdapter> AbsoluteModel;
  opengv::sac::Ransac<AbsoluteModel> ransac;
  std::shared_ptr<AbsoluteModel> absposeproblem_ptr(
        new AbsoluteModel(adapter, AbsoluteModel::Algorithm::GP3P));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 9;
  ransac.max_iterations_ = 50;
  // initial guess not needed...
  // run the ransac
  ransac.computeModel(0);

  // assign transformation
  /// \todo: why is the transformation not assigned...???
  numInliers = int(ransac.inliers_.size());
  if (numInliers >= 10 && double(ransac.inliers_.size())/double(numCorrespondences)>0.7) {
    // kick out outliers:
    std::vector<bool> inliers(numCorrespondences, false);
    for (size_t k = 0; k < ransac.inliers_.size(); ++k) {
      inliers.at(size_t(ransac.inliers_.at(k))) = true;
    }

    for (size_t k = 0; k < numCorrespondences; ++k) {
      if (!inliers[k]) {
        // get the landmark id:
        size_t camIdx = size_t(adapter.camIndex(k));
        size_t keypointIdx = size_t(adapter.keypointIndex(k));

        // remove observation
        estimator.removeObservation(StateId(currentFrame->id()), camIdx, keypointIdx);
      }
    }
  }
  return numInliers;
}

// Perform 2D/2D RANSAC.
int Frontend::runRansac2d2d(Estimator &estimator, const okvis::ViParameters& params,
                            uint64_t currentFrameId, uint64_t olderFrameId,
                            bool initializePose, bool removeOutliers, bool& rotationOnly) {
  // match 2d2d
  rotationOnly = false;
  const size_t numCameras = params.nCameraSystem.numCameras();

  int totalInlierNumber = 0;
  bool rotation_only_success = false;
  bool rel_pose_success = false;

  // run relative RANSAC
  for (size_t im = 0; im < numCameras; ++im) {
    // relative pose adapter for Kneip toolchain
    opengv::relative_pose::FrameRelativeAdapter adapter(estimator, params.nCameraSystem,
                                                        olderFrameId, im, currentFrameId, im);

    size_t numCorrespondences = adapter.getNumberCorrespondences();

    if (numCorrespondences < 10)
      continue;  // won't generate meaningful results. let's hope the few corresp. are inliers!!

    // try both the rotation-only RANSAC and the relative one:

    // create a RelativePoseSac problem and RANSAC
    typedef opengv::sac_problems::relative_pose::FrameRotationOnlySacProblem
        FrameRotationOnlySacProblem;
    opengv::sac::Ransac<FrameRotationOnlySacProblem> rotation_only_ransac;
    std::shared_ptr<FrameRotationOnlySacProblem> rotation_only_problem_ptr(
          new FrameRotationOnlySacProblem(adapter));
    rotation_only_ransac.sac_model_ = rotation_only_problem_ptr;
    rotation_only_ransac.threshold_ = 9;
    rotation_only_ransac.max_iterations_ = 50;

    // run the ransac
    rotation_only_ransac.computeModel(0);

    // get quality
    int rotation_only_inliers = int(rotation_only_ransac.inliers_.size());
    float rotation_only_ratio = float(rotation_only_inliers) / float(numCorrespondences);

    // now the rel_pose one:
    typedef opengv::sac_problems::relative_pose::FrameRelativePoseSacProblem
        FrameRelativePoseSacProblem;
    opengv::sac::Ransac<FrameRelativePoseSacProblem> rel_pose_ransac;
    std::shared_ptr<FrameRelativePoseSacProblem> rel_pose_problem_ptr(
          new FrameRelativePoseSacProblem(adapter, FrameRelativePoseSacProblem::STEWENIUS));
    rel_pose_ransac.sac_model_ = rel_pose_problem_ptr;
    rel_pose_ransac.threshold_ = 9;  //(1.0 - cos(0.5/600));
    rel_pose_ransac.max_iterations_ = 50;

    // run the ransac
    rel_pose_ransac.computeModel(0);

    // assess success
    int rel_pose_inliers = int(rel_pose_ransac.inliers_.size());
    float rel_pose_ratio = float(rel_pose_inliers) / float(numCorrespondences);

    // decide on success and fill inliers
    std::vector<bool> inliers(numCorrespondences, false);
    if (rotation_only_ratio > rel_pose_ratio || rotation_only_ratio > 0.8f) {
      if (rotation_only_inliers > 10) {
        rotation_only_success = true;
      }
      rotationOnly = true;
      totalInlierNumber += rotation_only_inliers;
      for (size_t k = 0; k < rotation_only_ransac.inliers_.size(); ++k) {
        inliers.at(size_t(rotation_only_ransac.inliers_.at(k))) = true;
      }
    } else {
      if (rel_pose_inliers > 10 && rel_pose_ratio > 0.8f) {
        rel_pose_success = true;
      }
      totalInlierNumber += rel_pose_inliers;
      for (size_t k = 0; k < rel_pose_ransac.inliers_.size(); ++k) {
        inliers.at(size_t(rel_pose_ransac.inliers_.at(k))) = true;
      }
    }

    // failure?
    if (!rotation_only_success && !rel_pose_success) {
      continue;
    }

    // otherwise: kick out outliers!
    std::shared_ptr<okvis::MultiFrame> multiFrame = estimator.multiFrame(StateId(currentFrameId));
    for (size_t k = 0; k < numCorrespondences; ++k) {
      size_t idxB = adapter.getMatchKeypointIdxB(k);
      if (removeOutliers && !inliers[k]) {
        uint64_t lmIdB = multiFrame->landmarkId(im, idxB);
        if(lmIdB !=0) {
          estimator.removeObservation(StateId(currentFrameId), im, idxB);
        }
      }
    }

    // initialize pose if necessary
    if (initializePose && !isInitialized_) {
      if (rel_pose_success) {
        //LOG(INFO) << "Initializing pose from 2D-2D RANSAC"; #Sebastian
      } else {
        //LOG(INFO) << "Initializing pose from 2D-2D RANSAC: orientation only";
      }

      Eigen::Matrix4d T_C1C2_mat = Eigen::Matrix4d::Identity();

      okvis::kinematics::Transformation T_SCA, T_WSA, T_SC0, T_WS0;
      uint64_t idA = olderFrameId;
      uint64_t id0 = currentFrameId;
      T_SCA = estimator.extrinsics(StateId(idA), uchar(im));
      T_WSA = estimator.pose(StateId(idA));
      T_SC0 = estimator.extrinsics(StateId(id0), uchar(im));
      T_WS0 = estimator.pose(StateId(id0));
      if (rel_pose_success) {
        // update pose: this is unit length, will scale with IMU prediction later
        T_C1C2_mat.topLeftCorner<3, 4>() = rel_pose_ransac.model_coefficients_;
        // this is the prediction by IMU
        okvis::kinematics::Transformation T_C1C2_imu_pred =
            T_SCA.inverse() * T_WSA.inverse() * T_WS0 * T_SC0;
        // now scale. TODO: check directions?
        const Eigen::Vector3d r_imu = T_C1C2_imu_pred.r();
        if (r_imu.norm() < 1.0) {
          T_C1C2_mat.topRightCorner<3, 1>() *= r_imu.norm();
        }
      } else {
        // rotation only assigned...
        T_C1C2_mat.topLeftCorner<3, 3>() = rotation_only_ransac.model_coefficients_;
      }
    }
  }

  if (rel_pose_success || rotation_only_success) {
    return totalInlierNumber;
  }

  rotationOnly = true;  // hack...
  return -1;

}

// (re)instantiates feature detectors and descriptor extractors. Used after settings changed or at
// startup.
void Frontend::initialiseBriskFeatureDetectors() {
  for (auto it = featureDetectorMutexes_.begin(); it != featureDetectorMutexes_.end(); ++it) {
    (*it)->lock();
  }
  //mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  featureDetectors_.clear();
  descriptorExtractors_.clear();
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectors_.push_back(std::shared_ptr<cv::FeatureDetector>(
        new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
            briskDetectionThreshold_, briskDetectionOctaves_,
            briskDetectionAbsoluteThreshold_, briskDetectionMaximumKeypoints_)));
    descriptorExtractors_.push_back(std::shared_ptr<cv::DescriptorExtractor>(
        new brisk::BriskDescriptorExtractor(
            briskDescriptionRotationInvariance_, briskDescriptionScaleInvariance_)));
  }
  for (auto it = featureDetectorMutexes_.begin(); it != featureDetectorMutexes_.end(); ++it) {
    (*it)->unlock();
  }
}

}  // namespace okvis
