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
 * @file Frontend.hpp
 * @brief Header file for the Frontend class.
 * @author Andreas Forster
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_FRONTEND_HPP_
#define INCLUDE_OKVIS_FRONTEND_HPP_

#include <mutex>

#include <okvis/assert_macros.hpp>
#include <okvis/ViSlamBackend.hpp>
#include <okvis/ViFrontendInterface.hpp>
#include <okvis/timing/Timer.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief A frontend using BRISK features
 */
class Frontend : public ViFrontendInterface {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor.
   * @param numCameras Number of cameras in the sensor configuration.
   * @param dBowVocDir The directory containing the DBoW vocabulary.
   */
  Frontend(size_t numCameras, std::string dBowVocDir);
  virtual ~Frontend() override;

  ///@{
  /**
   * @brief Detection and descriptor extraction on a per image basis.
   * @remark This method is threadsafe.
   * @param cameraIndex Index of camera to do detection and description.
   * @param frameOut    Multiframe containing the frames.
   *                    Resulting keypoints and descriptors are saved in here.
   * @param T_WC        Pose of camera with index cameraIndex at image capture time.
   * @param[in] keypoints If the keypoints are already available from a different source, provide
   *                      them here in order to skip detection.
   * @warning Using keypoints from a different source is not yet implemented.
   * @return True if successful.
   */
  virtual bool detectAndDescribe(size_t cameraIndex,
                                 std::shared_ptr<okvis::MultiFrame> frameOut,
                                 const okvis::kinematics::Transformation& T_WC,
                                 const std::vector<cv::KeyPoint> * keypoints) override final;

  /**
   * @brief Matching as well as initialization of landmarks and state.
   * @warning This method is not threadsafe.
   * @warning This method uses the estimator. Make sure to not access it in another thread.
   * @param estimator       Estimator.
   * @param params          Configuration parameters.
   * @param framesInOut     Multiframe including the descriptors of all the keypoints.
   * @param[out] asKeyframe Should the frame be a keyframe?
   * @return True if successful.
   */
  virtual bool dataAssociationAndInitialization(
      Estimator& estimator,
      const okvis::ViParameters & params,
      std::shared_ptr<okvis::MultiFrame> framesInOut, bool* asKeyframe) override final;

  /**
   * @brief Propagates pose, speeds and biases with given IMU measurements.
   * @see okvis::ceres::ImuError::propagation()
   * @remark This method is threadsafe.
   * @param[in] imuMeasurements All the IMU measurements.
   * @param[in] imuParams The parameters to be used.
   * @param[inout] T_WS_propagated Start pose.
   * @param[inout] speedAndBiases Start speed and biases.
   * @param[in] t_start Start time.
   * @param[in] t_end End time.
   * @param[out] covariance Covariance for GIVEN start states.
   * @param[out] jacobian Jacobian w.r.t. start states.
   * @return True on success.
   */
  virtual bool propagation(const okvis::ImuMeasurementDeque & imuMeasurements,
                           const okvis::ImuParameters & imuParams,
                           okvis::kinematics::Transformation& T_WS_propagated,
                           okvis::SpeedAndBias & speedAndBiases,
                           const okvis::Time& t_start, const okvis::Time& t_end,
                           Eigen::Matrix<double, 15, 15>* covariance,
                           Eigen::Matrix<double, 15, 15>* jacobian) const override final;

  ///@}
  /// @name Getters related to the BRISK detector
  /// @{

  /// @brief Get the number of octaves of the BRISK detector.
  size_t getBriskDetectionOctaves() const {
    return briskDetectionOctaves_;
  }

  /// @brief Get the detection threshold of the BRISK detector.
  double getBriskDetectionThreshold() const {
    return briskDetectionThreshold_;
  }

  /// @brief Get the absolute threshold of the BRISK detector.
  double getBriskDetectionAbsoluteThreshold() const {
    return briskDetectionAbsoluteThreshold_;
  }

  /// @brief Get the maximum amount of keypoints of the BRISK detector.
  size_t getBriskDetectionMaximumKeypoints() const {
    return briskDetectionMaximumKeypoints_;
  }

  ///@}
  /// @name Getters related to the BRISK descriptor
  /// @{

  /// @brief Get the rotation invariance setting of the BRISK descriptor.
  bool getBriskDescriptionRotationInvariance() const {
    return briskDescriptionRotationInvariance_;
  }

  /// @brief Get the scale invariance setting of the BRISK descriptor.
  bool getBriskDescriptionScaleInvariance() const {
    return briskDescriptionScaleInvariance_;
  }

  ///@}
  /// @name Other getters
  /// @{

  /// @brief Get the matching threshold.
  double getBriskMatchingThreshold() const {
    return briskMatchingThreshold_;
  }

  /// @brief Get the area overlap threshold under which a new keyframe is inserted.
  float getKeyframeInsertionOverlapThershold() const {
    return keyframeInsertionOverlapThreshold_;
  }

  /// @brief Returns true if the initialization has been completed (RANSAC with actual translation)
  bool isInitialized() {
    return isInitialized_;
  }

  /// @}
  /// @name Setters related to the BRISK detector
  /// @{

  /// @brief Set the number of octaves of the BRISK detector.
  void setBriskDetectionOctaves(size_t octaves) {
    briskDetectionOctaves_ = octaves;
    initialiseBriskFeatureDetectors();
  }

  /// @brief Set the detection threshold of the BRISK detector.
  void setBriskDetectionThreshold(double threshold) {
    briskDetectionThreshold_ = threshold;
    initialiseBriskFeatureDetectors();
  }

  /// @brief Set the absolute threshold of the BRISK detector.
  void setBriskDetectionAbsoluteThreshold(double threshold) {
    briskDetectionAbsoluteThreshold_ = threshold;
    initialiseBriskFeatureDetectors();
  }

  /// @brief Set the maximum number of keypoints of the BRISK detector.
  void setBriskDetectionMaximumKeypoints(size_t maxKeypoints) {
    briskDetectionMaximumKeypoints_ = maxKeypoints;
    initialiseBriskFeatureDetectors();
  }

  /// @}
  /// @name Setters related to the BRISK descriptor
  /// @{

  /// @brief Set the rotation invariance setting of the BRISK descriptor.
  void setBriskDescriptionRotationInvariance(bool invariance) {
    briskDescriptionRotationInvariance_ = invariance;
    initialiseBriskFeatureDetectors();
  }

  /// @brief Set the scale invariance setting of the BRISK descriptor.
  void setBriskDescriptionScaleInvariance(bool invariance) {
    briskDescriptionScaleInvariance_ = invariance;
    initialiseBriskFeatureDetectors();
  }

  ///@}
  /// @name Other setters
  /// @{

  /// @brief Set the matching threshold.
  void setBriskMatchingThreshold(double threshold) {
    briskMatchingThreshold_ = threshold;
  }

  /// @brief Set the area overlap threshold under which a new keyframe is inserted.
  void setKeyframeInsertionOverlapThreshold(float threshold) {
    keyframeInsertionOverlapThreshold_ = threshold;
  }

  /// @}

  /// \brief Stop all CNN background threads.
  void endCnnThreads();

  /// \brief Clears and resets everything (so you can re-start).
  void clear();

private:

  /**
   * @brief   feature detectors with the current settings.
   *          The vector contains one for each camera to ensure that there are no problems with
   *          parallel detection.
   * @warning Lock with featureDetectorMutexes_[cameraIndex] when using the detector.
   */
  std::vector<std::shared_ptr<cv::FeatureDetector> > featureDetectors_;
  /**
   * @brief   feature descriptors with the current settings.
   *          The vector contains one for each camera to ensure that there are no problems with
   *          parallel detection.
   * @warning Lock with featureDetectorMutexes_[cameraIndex] when using the descriptor.
   */
  std::vector<std::shared_ptr<cv::DescriptorExtractor> > descriptorExtractors_;
  /// Mutexes for feature detectors and descriptors.
  std::vector<std::unique_ptr<std::mutex> > featureDetectorMutexes_;

  bool isInitialized_;        ///< Is the pose initialised?
  const size_t numCameras_;   ///< Number of cameras in the configuration.

  /// @name BRISK detection parameters
  /// @{

  size_t briskDetectionOctaves_;            ///< The set number of brisk octaves.
  double briskDetectionThreshold_;          ///< The set BRISK detection threshold.
  double briskDetectionAbsoluteThreshold_;  ///< The set BRISK absolute detection threshold.
  size_t briskDetectionMaximumKeypoints_;   ///< The set maximum number of keypoints.

  /// @}
  /// @name BRISK descriptor extractor parameters
  /// @{

  bool briskDescriptionRotationInvariance_; ///< The set rotation invariance setting.
  bool briskDescriptionScaleInvariance_;    ///< The set scale invariance setting.

  ///@}
  /// @name BRISK matching parameters
  ///@{

  double briskMatchingThreshold_; ///< The set BRISK matching threshold.

  ///@}

  /**
   * @brief If the hull-area around all matched keypoints of the current frame (with existing
   *        landmarks)
   *        divided by the hull-area around all keypoints in the current frame is lower than
   *        this threshold it should be a new keyframe.
   * @see   doWeNeedANewKeyframe()
   */
  float keyframeInsertionOverlapThreshold_;  //0.55

  /**
   * @brief Decision whether a new frame should be keyframe or not, based on overlap heuristic.
   * @param estimator     const reference to the estimator.
   * @param currentFrame  Keyframe candidate.
   * @return True if it should be a new keyframe.
   */
  bool doWeNeedANewKeyframe(const Estimator& estimator,
                            std::shared_ptr<okvis::MultiFrame> currentFrame);

  /**
   * @brief Match a new multiframe to existing keyframes
   * @tparam MATCHING_ALGORITHM Algorithm to match new keypoints to existing landmarks
   * @warning As this function uses the estimator it is not threadsafe
   * @param      estimator              Estimator.
   * @param[in]  params                 Parameter struct.
   * @param[in]  currentFrameId         ID of the current frame that should be matched against
   *                                    keyframes.
   * @param[in]  loopClosureLandmarksToUseExclusively Use these landmarks exclusively, if supplied.
   * @return The number of matches in total.
   */
  template<class CAMERA_GEOMETRY>
  int matchToMap(Estimator& estimator,
                 const okvis::ViParameters& params,
                 const uint64_t currentFrameId,
                 const std::set<LandmarkId>* loopClosureLandmarksToUseExclusively = nullptr);

  /**
   * @brief Match the frames inside the multiframe to each other to initialise new landmarks.
   * @tparam MATCHING_ALGORITHM Algorithm to match new keypoints to existing landmarks.
   * @warning As this function uses the estimator it is not threadsafe.
   * @param estimator Estimator.
   * @param multiFrame Multiframe containing the frames to match.
   * @param params The VI parameters.
   * @param asKeyframe Whether this is a keyframe.
   */
  template<class CAMERA_GEOMETRY>
  void matchStereo(Estimator& estimator,
                   std::shared_ptr<okvis::MultiFrame> multiFrame,
                   const okvis::ViParameters& params,
                   bool asKeyframe);

  /**
   * @brief Perform 3D/2D RANSAC.
   * @warning As this function uses the estimator it is not threadsafe.
   * @param estimator       Estimator.
   * @param nCameraSystem   Camera configuration and parameters.
   * @param currentFrame    Frame with the new potential matches.
   * @param removeOutliers  Remove observation of outliers in estimator.
   * @return Number of inliers.
   */
  int runRansac3d2d(Estimator& estimator,
                    const okvis::cameras::NCameraSystem &nCameraSystem,
                    std::shared_ptr<okvis::MultiFrame> currentFrame,
                    bool removeOutliers);

  /**
   * @brief Perform 2D/2D RANSAC.
   * @warning As this function uses the estimator it is not threadsafe.
   * @param estimator         Estimator.
   * @param params            Parameter struct.
   * @param currentFrameId    ID of the new multiframe containing matches with the frame with ID
   *                          olderFrameId.
   * @param olderFrameId      ID of the multiframe to which the current frame has been matched
   *                          against.
   * @param initializePose    If the pose has not yet been initialised should the function try to
   *                          initialise it.
   * @param removeOutliers    Remove observation of outliers in estimator.
   * @param[out] rotationOnly Was the rotation only RANSAC model enough to explain the matches.
   * @return Number of inliers.
   */
  int runRansac2d2d(Estimator& estimator,
                    const okvis::ViParameters& params, uint64_t currentFrameId,
                    uint64_t olderFrameId, bool initializePose,
                    bool removeOutliers, bool &rotationOnly);

  /// (re)instantiates feature detectors and descriptor extractors. Used after settings changed or
  /// at startup.
  void initialiseBriskFeatureDetectors();

  /// \brief Classification network for keypoints (if enabled).
  std::vector<std::shared_ptr<Network>> networks_;

  /// \brief DBoW for loop closure
  /// https://en.cppreference.com/w/cpp/language/pimpl
  class DBoW;
  std::unique_ptr<DBoW> dBow_;

  /**
   * @brief Match the frames to older, co-visible frames and triangulate.
   * @tparam CAMERA_GEOMETRY The camera geometry type to use.
   * @warning As this function uses the estimator it is not threadsafe.
   * @param estimator Estimator.
   * @param params The VI parameters.
   * @param currentFrameId Current frame ID.
   * @param[out] rotationOnly Result obtained matches with rotation-only RANSAC.
   */
  template<class CAMERA_GEOMETRY>
  int matchMotionStereo(Estimator &estimator, const okvis::ViParameters &params,
                        const uint64_t currentFrameId, bool &rotationOnly);

  /**
   * @brief Parallelisable sub-part of matchToMap.
   * @tparam CAMERA_GEOMETRY The camera geometry type to use.
   * @param threadIdx Thread index.
   * @param numThreads Total number of threads to use.
   * @param estimator Estimator.
   * @param params The VI parameters.
   * @param currentFrameId Current frame ID.
   * @param loopClosureLandmarksToUseExclusively Only use these landmarks, if not nullptr.
   * @param T_WS1 Pose guess.
   * @param numKeypoints Number of keypoints (in image im)
   * @param pointMap Current map.
   * @param im The image idx.
   * @param multiFrame current multiFrame.
   * @param[out] distances Distances of landmarks.
   * @param[out] lmIds matched landmark IDs.
   * @param[out] hps_W matched landmarks (homogeneous) positions.
   * @param[out] ctrs Number of matches (by im).
   */
  template<class CAMERA_GEOMETRY>
  void matchToMapByThread(
      size_t threadIdx, size_t numThreads, const Estimator &estimator,
      const okvis::ViParameters& params, const uint64_t currentFrameId,
      const std::set<LandmarkId>* loopClosureLandmarksToUseExclusively,
      const kinematics::Transformation& T_WS1, size_t numKeypoints,
      const MapPoints& pointMap, size_t im, const MultiFramePtr&  multiFrame,
      std::vector<double>& distances, std::vector<LandmarkId>& lmIds,
      AlignedVector<Eigen::Vector4d>& hps_W, std::vector<size_t>& ctrs) const;

  std::atomic_bool trackingLost_; ///< Is the tracking currently lost?

  /// \brief Hacky: remember which CNNs are running on what frame.
  std::map<StateId,std::vector<std::thread*>> cnnThreads_;
};

}  // namespace okvis

#endif // INCLUDE_OKVIS_FRONTEND_HPP_
