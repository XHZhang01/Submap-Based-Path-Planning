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
 *  Created on: April 10, 2020
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file DatasetWriter.cpp
 * @brief Source file for the DatasetWriter class.
 * @author Stefan Leutenegger
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <okvis/DatasetWriter.hpp>

namespace okvis {

const std::string IMAGE_FILE_SUFFIX_PNG = ".png";
const std::string IMAGE_FILE_SUFFIX_JPG = ".jpg";
const std::string IMAGE_FILE_SUFFIX_TIF = ".tif";

bool setupImagesDirectory_(const size_t numCameras,
                           const std::string& datasetDirectory,
                           std::vector<std::string>& directories,
                           std::vector<std::ofstream>& csvs,
                           const std::string& sensorName);

bool addImages_(threadsafe::Queue<std::vector<okvis::CameraMeasurement> >& imageQueue,
                const okvis::Time & stamp,
                const std::vector<cv::Mat> & images,
                const std::string& sensorName);

void processImages_(threadsafe::Queue<std::vector<okvis::CameraMeasurement> >& imageQueue,
                    std::vector<std::string>& directories,
                    std::vector<std::ofstream>& csvs,
                    threadsafe::Queue<std::vector<okvis::CameraMeasurement> >& visualizationQueue,
                    const std::function<void(cv::Mat&)>& transform,
                    const std::string& suffix);

void displayFrames_(threadsafe::Queue<std::vector<okvis::CameraMeasurement> >& visualizationQueue,
                    const std::string& sensorName, cv::Mat & outImg);

void noImageTransform(cv::Mat& frame) {}
void expImageTransform(cv::Mat& frame) {
  cv::exp(-frame / 10000.f * std::log(2), frame);  // equivalent to 2^(-frame/10000)
}

DatasetWriter::DatasetWriter(ViParameters &parameters,
                             const std::string &path,
                             const bool enable_rgb,
                             const bool enable_depth)
  : parameters_(parameters){

  // check valid path
  OKVIS_ASSERT_TRUE(Exception, boost::filesystem::is_directory(path),
                    "provided path: " << path << " not a valid directory")

      // create dataset folder with timestamp
      auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  datasetDirectory_ << path << "/" << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  boost::filesystem::create_directory(datasetDirectory_.str());

  // create imu csv file
  boost::filesystem::create_directory(datasetDirectory_.str() + "/imu0/");
  imuDirectory_ << datasetDirectory_.str() << "/imu0/";
  imuCsv_.open(imuDirectory_.str() + "data.csv");
  OKVIS_ASSERT_TRUE(Exception, imuCsv_.good(), "couldn't open " << imuDirectory_.str() << "data.csv")
  imuCsv_ << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"<<std::endl;

  // create image folders, CSVs and image queues
  setupImagesDirectory_(parameters.nCameraSystem.numCameras(),
                        datasetDirectory_.str(),
                        camDirectories_,
                        camCsvs_,
                        "cam");

  // start processing thread
  shutdown_ = false;
  imuProcessingThread_ = std::thread(&DatasetWriter::processingImu, this);
  imagesProcessingThread_ = std::thread(&DatasetWriter::processingImages, this);

  // create RGB folder and start processing thread, if enabled.
  if(enable_rgb) {
    setupImagesDirectory_(1,
                          datasetDirectory_.str(),
                          rgbCamDirectories_,
                          rgbCamCsvs_,
                          "rgb");
    rgbProcessingThread_ = std::thread(&DatasetWriter::processingRGBImages, this);
  }

  // create depth folder and start processing thread, if enabled.
  if(enable_depth) {
    setupImagesDirectory_(1,
                          datasetDirectory_.str(),
                          depthCamDirectories_,
                          depthCamCsvs_,
                          "depth");
    depthProcessingThread_ = std::thread(&DatasetWriter::processingDepthImages, this);
  }
}

DatasetWriter::~DatasetWriter() {

  // shutdown queues
  imuMeasurementsReceived_.Shutdown();
  cameraMeasurementsReceived_.Shutdown();
  rgbCameraMeasurementsReceived_.Shutdown();
  depthCameraMeasurementsReceived_.Shutdown();
  visualisations_.Shutdown();
  visualisationsRGB_.Shutdown();
  visualisationsDepth_.Shutdown();

  // finish writing what's already in the queues
  shutdown_ = true;
  imuProcessingThread_.join();
  imagesProcessingThread_.join();
  if(rgbProcessingThread_.joinable())
    rgbProcessingThread_.join();
  if(depthProcessingThread_.joinable())
    depthProcessingThread_.join();

  // close CSV files
  imuCsv_.close();
  for(size_t i = 0; i<camCsvs_.size(); ++i) {
    camCsvs_.at(i).close();
  }
  for(size_t i = 0; i<rgbCamCsvs_.size(); ++i) {
    rgbCamCsvs_.at(i).close();  // close can be called even if never opened
  }
  for(size_t i = 0; i<depthCamCsvs_.size(); ++i) {
    depthCamCsvs_.at(i).close();  // close can be called even if never opened
  }
}

bool DatasetWriter::addImages(const Time &stamp, const std::vector<cv::Mat> &images) {
  return addImages_(cameraMeasurementsReceived_, stamp, images, "cam");
}

bool DatasetWriter::addRGBImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) {
  if(!rgbProcessingThread_.joinable()) {
    return false;
  }
  return addImages_(rgbCameraMeasurementsReceived_, stamp, images, "rgb");
}

bool DatasetWriter::addRGBImage(const okvis::Time & stamp, const cv::Mat & image) {
  std::vector<cv::Mat> images = {image};
  return addRGBImages(stamp, images);
}

bool DatasetWriter::addDepthImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) {
  if(!depthProcessingThread_.joinable()) {
    return false;
  }
  return addImages_(depthCameraMeasurementsReceived_, stamp, images, "depth");
}

bool DatasetWriter::addDepthImage(const okvis::Time & stamp, const cv::Mat & image) {
  std::vector<cv::Mat> images = {image};
  return addDepthImages(stamp, images);
}

bool DatasetWriter::addImuMeasurement(const Time &stamp, const Eigen::Vector3d &alpha, const Eigen::Vector3d &omega) {
  okvis::ImuMeasurement imu_measurement;
  imu_measurement.measurement.accelerometers = alpha;
  imu_measurement.measurement.gyroscopes = omega;
  imu_measurement.timeStamp = stamp;

  const int imuQueueSize = 100;
  if(imuMeasurementsReceived_.PushNonBlockingDroppingIfFull(imu_measurement, size_t(imuQueueSize))) {
    LOG(WARNING) << "imu measurement drop ";
    return false;
  }
  return true;
}

void DatasetWriter::setBlocking(bool blocking) {
  if(blocking) {
    LOG(WARNING) << "DatasetWriter cannot run in blocking mode" << std::endl;
  }
}

void DatasetWriter::display(cv::Mat & images, cv::Mat & topDebugImg) {
  cv::Mat outRgb, outDepth;
  displayFrames_(visualisations_, "cam", images);
  displayFrames_(visualisationsRGB_, "rgb", outRgb);
  displayFrames_(visualisationsDepth_, "depth", outDepth);
  if(images.empty()){
    images = outRgb;
  } else {
    cv::vconcat(images, outRgb, images);
  }
  if(images.empty()){
    images = outDepth;
  } else {
    cv::vconcat(images, outDepth, images);
  }
}

void DatasetWriter::processingImu() {
  while(!shutdown_) {
    ImuMeasurement imuMeasurement;
    while(imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement)) {
      imuCsv_ << imuMeasurement.timeStamp.toNSec() << ","
              << imuMeasurement.measurement.gyroscopes[0] << ","
              << imuMeasurement.measurement.gyroscopes[1] << ","
              << imuMeasurement.measurement.gyroscopes[2] << ","
              << imuMeasurement.measurement.accelerometers[0] << ","
              << imuMeasurement.measurement.accelerometers[1] << ","
              << imuMeasurement.measurement.accelerometers[2] <<std::endl;
    }
  }
}

void DatasetWriter::processingImages() {
  while(!shutdown_)
    processImages_(cameraMeasurementsReceived_,
                   camDirectories_,
                   camCsvs_,
                   visualisations_,
                   noImageTransform,
                   IMAGE_FILE_SUFFIX_PNG);
}

void DatasetWriter::processingRGBImages() {
  // In opposite to IR images RGB images have multiple channels and typically have a larger
  // resolution. Storing them as .png would take too long. .jpg files on the other side
  // are much more efficient to store, as the writer is IO-bound (compression overhead <<
  // IO cpu time).
  while(!shutdown_)
    processImages_(rgbCameraMeasurementsReceived_,
                   rgbCamDirectories_,
                   rgbCamCsvs_,
                   visualisationsRGB_,
                   noImageTransform,
                   IMAGE_FILE_SUFFIX_JPG);
}

void DatasetWriter::processingDepthImages() {
  while(!shutdown_)
    processImages_(depthCameraMeasurementsReceived_,
                   depthCamDirectories_,
                   depthCamCsvs_,
                   visualisationsDepth_,
                   expImageTransform,
                   IMAGE_FILE_SUFFIX_TIF);
}

bool setupImagesDirectory_(const size_t numCameras,
                           const std::string& datasetDirectory,
                           std::vector<std::string>& directories,
                           std::vector<std::ofstream>& csvs,
                           const std::string& sensorName) {
  for(size_t i = 0; i<numCameras; ++i) {
    std::stringstream camDirectory;
    camDirectory << datasetDirectory << "/" << sensorName << i << "/";
    directories.push_back(camDirectory.str());
    boost::filesystem::create_directory(camDirectory.str());
    boost::filesystem::create_directory(camDirectory.str() + "data/");

    csvs.push_back(std::ofstream(camDirectory.str()+"data.csv"));
    OKVIS_ASSERT_TRUE(DatasetWriter::Exception, csvs.back().good(), "couldn't open " << camDirectory.str() << "data.csv");
    csvs.back() << "#timestamp [ns],filename" << std::endl;
  }
  return true;
}

bool addImages_(threadsafe::Queue<std::vector<okvis::CameraMeasurement> >& imageQueue,
                const okvis::Time & stamp,
                const std::vector<cv::Mat> & images,
                const std::string& sensorName) {
  // assemble frame
  const size_t numCameras = images.size();
  std::vector<okvis::CameraMeasurement> frames(numCameras);
  for(size_t i=0; i<numCameras; ++i) {
    frames.at(i).measurement.image = images.at(i);
    frames.at(i).timeStamp = stamp;
    frames.at(i).sensorId = int(i);
    frames.at(i).measurement.deliversKeypoints = false;
  }

  const int cameraInputQueueSize = 100;
  if(imageQueue.PushNonBlockingDroppingIfFull(frames, cameraInputQueueSize)) {
    LOG(WARNING) << sensorName << ": frame drop";
    return false;
  }
  return true;
}

void processImages_(threadsafe::Queue<std::vector<okvis::CameraMeasurement> >& imageQueue,
                    std::vector<std::string>& directories,
                    std::vector<std::ofstream>& csvs,
                    threadsafe::Queue<std::vector<okvis::CameraMeasurement> >& visualizationQueue,
                    const std::function<void(cv::Mat&)>& transform,
                    const std::string& suffix) {
  std::vector<CameraMeasurement> frames;
  while(imageQueue.PopNonBlocking(&frames)) {
    const size_t numCameras = frames.size();
    for (size_t i = 0; i < numCameras; ++i) {
      // write text file data
      uint64_t timestamp = frames.at(i).timeStamp.toNSec();
      csvs.at(i) << timestamp << "," << timestamp << suffix << std::endl;

      // write image
      std::stringstream imagename;
      imagename << directories.at(i) << "data/" << timestamp << suffix;
      transform(frames.at(i).measurement.image);
      cv::imwrite(imagename.str(), frames.at(i).measurement.image);
    }
    visualizationQueue.PushNonBlockingDroppingIfFull(frames, 1);
  }
}

void displayFrames_(threadsafe::Queue<std::vector<okvis::CameraMeasurement> >& visualizationQueue,
                    const std::string& sensorName, cv::Mat& outImg) {
  std::vector<CameraMeasurement> frames;
  if(visualizationQueue.PopNonBlocking(&frames)) {
    for(size_t i=0; i<frames.size(); ++i) {
      if(i==0){
        outImg = frames.at(i).measurement.image;
      } else {
        cv::hconcat(outImg, frames.at(i).measurement.image, outImg);
      }
    }
  }
}
}
