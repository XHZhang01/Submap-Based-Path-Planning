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
 * @file DatasetReader.cpp
 * @brief Source file for the DatasetReader class.
 * @author Stefan Leutenegger
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <okvis/DatasetReader.hpp>

namespace okvis {

DatasetReader::DatasetReader(const std::string & path,
                             const Duration & deltaT, int numCameras, const GpsParameters& gpsParameters) :
  deltaT_(deltaT), numCameras_(numCameras), gpsFlag_(gpsParameters.use_gps), gpsDataType_(gpsParameters.type) {
  OKVIS_ASSERT_TRUE(Exception, gpsDataType_=="cartesian" || gpsDataType_=="geodetic" || gpsDataType_=="geodetic-leica",
                    "Unknown GPS data type specified")
  streaming_ = false;
  setDatasetPath(path);
  counter_ = 0;
  t_gps_ = okvis::Time(0.0);
}

DatasetReader::~DatasetReader() {
  stopStreaming();
}

bool DatasetReader::setDatasetPath(const std::string & path) {
  path_ = path;
  return true;
}

bool DatasetReader::setStartingDelay(const Duration &deltaT)
{
  if(streaming_) {
    LOG(WARNING)<< "starting delay ignored, because streaming already started";
    return false;
  }
  deltaT_ = deltaT;
  return true;
}

bool DatasetReader::isStreaming()
{
  return streaming_;
}

double DatasetReader::completion() const {
  if(streaming_) {
    return double(counter_)/double(numImages_);
  }
  return 0.0;
}

bool DatasetReader::startStreaming() {
  OKVIS_ASSERT_TRUE(Exception, imagesCallback_, "no add image callback registered")
  OKVIS_ASSERT_TRUE(Exception, imuCallback_, "no add IMU callback registered")
  if(gpsFlag_)
          OKVIS_ASSERT_TRUE(Exception, geodeticGpsCallback_ || gpsCallback_, "no add GPS callback registered")

  // open the IMU file
  std::string line;
  imuFile_.open(path_ + "/imu0/data.csv");
  OKVIS_ASSERT_TRUE(Exception, imuFile_.good(), "no imu file found at " << path_+"/imu0/data.csv");
  int number_of_lines = 0;
  while (std::getline(imuFile_, line))
    ++number_of_lines;
  LOG(INFO)<< "No. IMU measurements: " << number_of_lines-1;
  if (number_of_lines - 1 <= 0) {
    LOG(ERROR)<< "no imu messages present in " << path_+"/imu0/data.csv";
    return -1;
  }
  // set reading position to second line
  imuFile_.clear();
  imuFile_.seekg(0, std::ios::beg);
  std::getline(imuFile_, line);

  if(gpsFlag_)
  {
      // open the GPS file
      std::string gline;
      if(gpsDataType_ =="cartesian")
        gpsFile_.open(path_ + "/gps0/data.csv");
      else if(gpsDataType_=="geodetic")
        gpsFile_.open(path_ + "/gps0/data_raw.csv");
      else if(gpsDataType_ == "geodetic-leica")
        gpsFile_.open(path_+"gnss.csv");
      OKVIS_ASSERT_TRUE(Exception, gpsFile_.good(), "no gps file found at " << path_+"/gps0/");
      int gnumber_of_lines = 0;
      while (std::getline(gpsFile_, gline))
        ++gnumber_of_lines;
      LOG(INFO)<< "No. GPS measurements: " << gnumber_of_lines-1;
      if (gnumber_of_lines - 1 <= 0) {
        LOG(ERROR)<< "no gps messages present in " << path_+"/gps0/data.csv";
        return -1;
      }
      // set reading position to second line
      gpsFile_.clear();
      gpsFile_.seekg(0, std::ios::beg);
      std::getline(gpsFile_, gline);

  }

  // now open camera files
  std::vector<okvis::Time> times;
  okvis::Time latest(0);
  int i = 0;
  while (i<numCameras_) {
    std::vector < std::pair<std::string, std::string> > imageNames;
    int num_camera_images = readCameraImageCsv(path_ + "/cam" + std::to_string(i) + "/data.csv", imageNames);
    if (num_camera_images < 0) {
      OKVIS_ASSERT_TRUE(Exception, i>0, "No camera data found");
      break;
    }
    LOG(INFO)<< "No. cam " << i << " images: " << num_camera_images;
    if(i==0) {
      numImages_ = num_camera_images;
    }
    allImageNames_.push_back(imageNames);
    ++i; // try next camera
  }

  // open rgb camera files, if they exist.
  int num_camera_images = readCameraImageCsv(path_ + "/rgb0/data.csv", rgbImageNames_);
  LOG(INFO)<< "RGB cam " << i << " images: " << num_camera_images;

  counter_ = 0;
  streaming_ = true;
  processingThread_ = std::thread(&DatasetReader::processing, this);

  return true;
}

int DatasetReader::readCameraImageCsv(
    const std::string& filename,
    std::vector < std::pair<std::string, std::string> >& imageNames) const
{
  std::ifstream camDataFile(filename);
  std::string line;
  if(!camDataFile.good()) {
    return -1;
  }
  int num_camera_images = 0;
  std::getline(camDataFile, line);
  while (std::getline(camDataFile, line)) {
    ++num_camera_images;
    std::stringstream stream(line);
    std::string s0, s1;
    std::getline(stream, s0, ',');
    imageNames.push_back(std::make_pair(s0,s0+".png"));
  }
  return num_camera_images;
}

bool DatasetReader::stopStreaming() {
  // Stop the pipeline
  if(processingThread_.joinable()) {
    processingThread_.join();
    streaming_ = false;
  }
  return true;
}

void  DatasetReader::processing() {
  std::string line;
  okvis::Time start(0.0);
  const size_t numCameras = allImageNames_.size();
  std::vector < std::vector < std::pair<std::string, std::string> > ::iterator
      > cam_iterators(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    cam_iterators.at(i) = allImageNames_.at(i).begin();
  }
  std::vector<std::pair<std::string, std::string>> ::iterator rgbIterator;
  rgbIterator = rgbImageNames_.begin();
  const uint64_t tolNSec = 10000000; // 0.01 sec
  while (streaming_) {

    // sync and check if at the end
    bool synched = false;
    while(!synched) {
      uint64_t max_timestamp = 0;
      for (size_t i = 0; i < numCameras; ++i) {
        if(cam_iterators.at(i) == allImageNames_.at(i).end()) {
          streaming_ = false;
          return; // finished!
        }
        uint64_t tcam = std::atol(cam_iterators.at(i)->first.c_str());
        if(tcam>max_timestamp) {
          max_timestamp = tcam;
        }
      }
      synched=true;
      for (size_t i = 0; i < numCameras; ++i) {
        uint64_t tcam = std::atol(cam_iterators.at(i)->first.c_str());
        if(tcam < max_timestamp - tolNSec) {
          cam_iterators.at(i)++; // not in tolerance, advance;
          synched = false;
        }
      }
    }

    // check synched (I am paranoid)
    for (size_t i = 0; i < numCameras; ++i) {
      if(i>0) {
        int64_t tcam = std::atol(cam_iterators.at(0)->first.c_str());
        int64_t tcam_i = std::atol(cam_iterators.at(i)->first.c_str());
        OKVIS_ASSERT_TRUE(Exception, abs(tcam-tcam_i)<int64_t(tolNSec),
                          "timestamp mismatch " << double(abs(tcam-tcam_i))/1.0e6 << " ms at " << tcam_i);
      }
    }

    // add images
    okvis::Time t;
    std::vector<cv::Mat> images(numCameras);
    for (size_t i = 0; i < numCameras; ++i) {

      std::string filename = path_ + "/cam" + std::to_string(i) + "/data/" + cam_iterators.at(i)->second;
      cv::Mat filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE);

      OKVIS_ASSERT_TRUE(
            Exception, !filtered.empty(),
            "cam " << i << " missing image :" << std::endl << filename)

      t.fromNSec(std::atol(cam_iterators.at(i)->first.c_str()));

      if (start == okvis::Time(0.0)) {
        start = t;
      }

      // get all IMU measurements till then
      okvis::Time t_imu = start;
      do {
        if (!std::getline(imuFile_, line)) {
          streaming_ = false;
          return;
        }

        std::stringstream stream(line);
        std::string s;
        std::getline(stream, s, ',');
        uint64_t nanoseconds = std::stol(s.c_str());

        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ',');
          gyr[j] = std::stof(s);
        }

        Eigen::Vector3d acc;
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ',');
          acc[j] = std::stof(s);
        }

        t_imu.fromNSec(nanoseconds);

        // add the IMU measurement for (blocking) processing
        if (t_imu - start + okvis::Duration(1.0) > deltaT_) {
          imuCallback_(t_imu, acc, gyr);
        }

      } while (t_imu <= t);

      if(gpsFlag_){

          std::string gline;
          while(t_gps_ <= t)/*do*/ {
            if (!std::getline(gpsFile_, gline)) {
              streaming_ = false;
              return;
            }

            std::stringstream gstream(gline);
            std::string gs;

            // Distinguish GPS data type
            if(gpsDataType_ == "cartesian"){

              std::getline(gstream, gs, ',');
              uint64_t gnanoseconds = std::stol(gs.c_str());

              Eigen::Vector3d pos;
              for (int j = 0; j < 3; ++j) {
                std::getline(gstream, gs, ',');
                pos[j] = std::stof(gs);
              }

              Eigen::Vector3d err;
              for (int j = 0; j < 3; ++j) {
                std::getline(gstream, gs, ',');
                err[j] = std::stof(gs);
              }

              t_gps_.fromNSec(gnanoseconds);

              // add the GPS measurement for (blocking) processing
              if (t_gps_ - start + okvis::Duration(1.0) > deltaT_) {
                gpsCallback_(t_gps_, pos, err);
              }

            }
            else if (gpsDataType_ == "geodetic"){

              // 1st entry timestamp
              std::getline(gstream, gs, ',');
              uint64_t gnanoseconds = std::stol(gs.c_str());

              // 2nd entry latitude
              std::getline(gstream, gs, ',');
              double lat = std::stod(gs.c_str());

              // 3rd entry longitude
              std::getline(gstream, gs, ',');
              double lon = std::stod(gs.c_str());

              // 4th entry height / altitude
              std::getline(gstream, gs, ',');
              double alt = std::stod(gs.c_str());

              // 5th horizontal error
              std::getline(gstream, gs, ',');
              double hErr = std::stod(gs.c_str());

              // 6th horizontal error
              std::getline(gstream, gs, ',');
              double vErr = std::stod(gs.c_str());

              t_gps_.fromNSec(gnanoseconds);

              // add the GPS measurement for (blocking) processing
              if (t_gps_ - start + okvis::Duration(1.0) > deltaT_) {
                geodeticGpsCallback_(t_gps_, lat, lon, alt, hErr, vErr);
              }
            }
            else if (gpsDataType_ == "geodetic-leica"){

              // 1st entry timestamp
              std::getline(gstream, gs, ',');
              uint64_t gnanoseconds = std::stol(gs.c_str());

              // 2nd entry date
              std::getline(gstream, gs, ',');
              // 3rd entry time
              std::getline(gstream, gs, ',');
              // 4th entry fix
              std::getline(gstream, gs, ',');
              // 5th entry rtk
              std::getline(gstream, gs, ',');
              // 6th entry num_sv
              std::getline(gstream, gs, ',');

              // 7th entry latitude
              std::getline(gstream, gs, ',');
              double lat = std::stod(gs.c_str());

              // 8th entry longitude
              std::getline(gstream, gs, ',');
              double lon = std::stod(gs.c_str());

              // 9th entry height / altitude
              std::getline(gstream, gs, ',');
              double alt = std::stod(gs.c_str());

              // 10th entry hmsl
              std::getline(gstream, gs, ',');

              // 11th horizontal error
              std::getline(gstream, gs, ',');
              double hErr = std::stod(gs.c_str());

              // 12th horizontal error
              std::getline(gstream, gs, ',');
              double vErr = std::stod(gs.c_str());

              t_gps_.fromNSec(gnanoseconds);

              // add the GPS measurement for (blocking) processing
              if (t_gps_ - start + okvis::Duration(1.0) > deltaT_) {
                geodeticGpsCallback_(t_gps_, lat, lon, alt, hErr, vErr);
              }
            }

          } /*while (t_gps <= t);*/


      }

      // add the image to the frontend for (blocking) processing
      if (t - start > deltaT_) {
        images.at(i) = filtered;
      }

      cam_iterators[i]++;

    }
    bool add=true;
    for (size_t i = 0; i < numCameras; ++i) {
      if(images.at(i).empty()) {
        add=false;
      }
    }
    if(add) {
      imagesCallback_(t, images);
      ++counter_;
    }

    if(rgbCallback_ && !rgbImageNames_.empty()) {
      int64_t tcam = std::atol(cam_iterators.at(0)->first.c_str());
      int64_t tcam_rgb = std::atol(rgbIterator->first.c_str());
      while (tcam_rgb < tcam && rgbIterator != rgbImageNames_.end()) {
        okvis::Time t_rgb;
        t_rgb.fromNSec(tcam_rgb);
        std::string filename = path_ + "/rgb0/data/" + rgbIterator->second;
        cv::Mat rgb_image = cv::imread(filename, cv::IMREAD_COLOR);
        rgbCallback_(t_rgb, rgb_image);
        rgbIterator++;
        tcam_rgb = std::atol(rgbIterator->first.c_str());
      }
    }
  }

  return;
}

}
