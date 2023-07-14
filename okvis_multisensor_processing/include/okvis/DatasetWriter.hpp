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
 *              Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file DatasetWriter.hpp
 * @brief Header file for the DatasetWriter class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_DATASETWRITER_HPP_
#define INCLUDE_OKVIS_DATASETWRITER_HPP_

#include <thread>
#include <atomic>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <fstream>

#include <boost/filesystem.hpp>

#include <okvis/Measurements.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/assert_macros.hpp>

#include <okvis/timing/Timer.hpp>
#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <okvis/ViInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief Dataset writer class.
class DatasetWriter : public ViInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// @brief Disallow default construction.
  DatasetWriter() = delete;

  /**
   * \brief Constructor.
   * \param parameters Parameters and settings.
   * \param path Path to dataset folder.
   * \param enable_rgb Enable RGB image writer.
   * \param enable_depth Enable depth image write.
   */
  DatasetWriter(okvis::ViParameters& parameters,
                const std::string& path = "",
                const bool enable_rgb = false,
                const bool enable_depth = false);

  /// \brief Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
  virtual ~DatasetWriter();

  /// \name Add measurements to the algorithm.
  /// \{
  /**
   * \brief              Add a set of new image.
   * \param stamp        The image timestamp.
   * \param images       The images.
   * \return             Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) final;

  /// \name Add a RGB measurements to the algorithm.
  /// \{
  /**
   * \brief              Add a set of new RGB images.
   * \param stamp        The image timestamp.
   * \param images       The images.
   * \return             Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addRGBImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) final;

  /// \name Add a single RGB measurement to the algorithm.
  /// \{
  /**
   * \brief              Add a single new RGB image.
   * \param stamp        The image timestamp.
   * \param image        The image.
   * \return             Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addRGBImage(const okvis::Time & stamp, const cv::Mat & image) final;

  /// \name Add a depth measurements to the algorithm.
  /// \{
  /**
   * \brief              Add a set of new RGB images.
   * \param stamp        The image timestamp.
   * \param images       The images.
   * \return             Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addDepthImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) final;

  /// \name Add a single depth measurement to the algorithm.
  /// \{
  /**
   * \brief              Add a single new depth image.
   * \param stamp        The image timestamp.
   * \param image        The image.
   * \return             Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addDepthImage(const okvis::Time & stamp, const cv::Mat & image) final;


  /**
   * \brief          Add an IMU measurement.
   * \param stamp    The measurement timestamp.
   * \param alpha    The acceleration measured at this time.
   * \param omega    The angular velocity measured at this time.
   * \return Returns true normally. False if the previous one has not been processed yet.
   */
  virtual bool addImuMeasurement(const okvis::Time & stamp,
                                 const Eigen::Vector3d & alpha,
                                 const Eigen::Vector3d & omega) final;

  /// \brief Indicats whether the add functions block. This only supports blocking.
  virtual void setBlocking(bool blocking) override;

  /// @brief Display some visualisation.
  virtual void display(cv::Mat & images, cv::Mat & topDebugImg) override final;

 private:

  /// @brief Processing loops and according threads.
  void processingImu();
  void processingImages();
  void processingRGBImages();
  void processingDepthImages();

  std::atomic_bool shutdown_; ///< True if shutdown requested.
  std::thread imuProcessingThread_;
  std::thread imagesProcessingThread_;
  std::thread rgbProcessingThread_;
  std::thread depthProcessingThread_;

  /// @brief Camera measurement input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > cameraMeasurementsReceived_;
  /// @brief Visualisation (live video) input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > visualisations_;
  /// @brief RGB camera measurement input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > rgbCameraMeasurementsReceived_;
  /// @brief RGB Visualisation (live video) input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > visualisationsRGB_;
  /// @brief Depth camera measurement input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > depthCameraMeasurementsReceived_;
  /// @brief Depth Visualisation (live video) input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > visualisationsDepth_;
  /// @brief IMU measurement input queue.
  threadsafe::Queue<okvis::ImuMeasurement, Eigen::aligned_allocator<okvis::ImuMeasurement>>
      imuMeasurementsReceived_;

  okvis::ViParameters parameters_; ///< All VI parameters

  std::stringstream datasetDirectory_; ///< Path to the dataset.
  std::stringstream imuDirectory_; ///< Directory to the IMU data file.
  std::ofstream imuCsv_; ///< IMU data file.
  std::vector<std::string> camDirectories_; ///< Directories to the camera data.
  std::vector<std::ofstream> camCsvs_; ///< Camera data files.
  std::vector<std::string> rgbCamDirectories_; ///< Directories to the RGB camera data.
  std::vector<std::ofstream> rgbCamCsvs_; ///< RGB data files.
  std::vector<std::string> depthCamDirectories_; ///< Directories to the depth camera data.
  std::vector<std::ofstream> depthCamCsvs_; ///< Depth data files.

};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_THREADEDSLAM3_HPP_ */
