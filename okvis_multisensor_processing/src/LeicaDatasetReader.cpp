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
 * @file LeicaDatasetReader.cpp
 * @brief Source file for the LeicaDatasetReader class.
 * @author Simon Boche
 */

#include <okvis/LeicaDatasetReader.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


namespace okvis {

    LeicaDatasetReader::LeicaDatasetReader(const std::string & path, const Duration & deltaT,
                                           bool useBottom, bool useFront, bool useLeft, bool useRight, bool useTop,
                                           const std::string cam_type, GpsParameters gpsParameters) :
            deltaT_(deltaT), useBottom_(useBottom), useFront_(useFront), useLeft_(useLeft),
            useRight_(useRight), useTop_(useTop),
            camType_(cam_type), gpsFlag_(gpsParameters.use_gps), gpsDataType_(gpsParameters.type){
      streaming_ = false;
      setDatasetPath(path);
      counter_ = 0;
      numCameras_ = useBottom_ + useTop_ + useLeft_ + useRight_ + useFront_;
      std::cout << "[Cam Info] Using " << numCameras_ << " cameras." << std::endl;
    }

    LeicaDatasetReader::~LeicaDatasetReader() {
      stopStreaming();
    }

    bool LeicaDatasetReader::setDatasetPath(const std::string & path) {
      path_ = path;
      return true;
    }

    bool LeicaDatasetReader::setStartingDelay(const Duration &deltaT)
    {
      if(streaming_) {
        LOG(WARNING)<< "starting delay ignored, because streaming already started";
        return false;
      }
      deltaT_ = deltaT;
      return true;
    }

    bool LeicaDatasetReader::isStreaming()
    {
      return streaming_;
    }

    double LeicaDatasetReader::completion() const {
      if(streaming_) {
        //double comp = double(counter_) / double(numImages_);
        //LOG(INFO) << std::to_string(comp);
        return double(counter_)/double(numImages_);
      }
      return 0.0;
    }

    bool LeicaDatasetReader::startStreaming() {
      //OKVIS_ASSERT_TRUE(Exception, imagesCallback_, "no add image callback registered")
      //OKVIS_ASSERT_TRUE(Exception, imuCallback_, "no add IMU callback registered")
      if(gpsFlag_)
        OKVIS_ASSERT_TRUE(Exception, geodeticGpsCallback_ || gpsCallback_, "no add GPS callback registered")
      // ToDo: Think about lidarCallback_ if Lidar is fused

      // Open LiDAR file
      std::string lidarLine;
      lidarFile_.open(path_ + "/lidar.csv");
      OKVIS_ASSERT_TRUE(Exception, lidarFile_.good(), "no lidar file found at " << path_+"/lidar.csv");
      int number_of_lidar_lines = 0;
      while (std::getline(lidarFile_, lidarLine))
        ++number_of_lidar_lines;
      LOG(INFO)<< "No. LiDAR measurements: " << number_of_lidar_lines-1;
      if (number_of_lidar_lines - 1 <= 0) {
        LOG(ERROR)<< "no lidar messages present in " << path_+"/lidar.csv";
        return -1;
      }
      // set reading position to second line
      lidarFile_.clear();
      lidarFile_.seekg(0, std::ios::beg);
      std::getline(lidarFile_, lidarLine);

      // open the IMU file
      std::string line;
      imuFile_.open(path_ + "/imu_bottom.csv"); // ToDo: Clarify which IMU is used, but bottom one should be fixed
      OKVIS_ASSERT_TRUE(Exception, imuFile_.good(), "no imu file found at " << path_+"/imu_bottom.csv");
      int number_of_lines = 0;
      while (std::getline(imuFile_, line))
        ++number_of_lines;
      LOG(INFO)<< "No. IMU measurements: " << number_of_lines-1;
      if (number_of_lines - 1 <= 0) {
        LOG(ERROR)<< "no imu messages present in " << path_+"/imu_bottom.csv";
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

      // now open camera files (read all cameras: bottom, top, front, left, right)
      std::vector<okvis::Time> times;
      okvis::Time latest(0);

      /** Convention for indices of the 5 cameras
       * 0: bottom
       * 1: front
       * 2: left
       * 3: right
       * 4: top
       */

      std::string imgPath;
      if(camType_ == "pinhole")
        imgPath = path_ + "/pinhole/";
      else if(camType_ == "eucm")
        imgPath = path_+"/imgs/";

      boost::filesystem::path camPath(imgPath);
      if(exists(camPath)){ // check if camera path exists
        if(is_directory(camPath)){ // check if it is a directory

          std::vector < std::pair<uint64_t , std::string> > bottomImageNames;
          std::vector < std::pair<uint64_t, std::string> > frontImageNames;
          std::vector < std::pair<uint64_t, std::string> > leftImageNames;
          std::vector < std::pair<uint64_t, std::string> > rightImageNames;
          std::vector < std::pair<uint64_t, std::string> > topImageNames;

          for(boost::filesystem::directory_entry& img : boost::filesystem::directory_iterator(camPath)){ // iterate images

            std::string img_name = img.path().stem().string(); // stem removes path and file extension
            // split image string into timestamp and camera name
            std::vector<std::string> strs;
            boost::split(strs, img_name, boost::is_any_of("_"));

            std::string timestring = strs.at(1);
            uint64_t timestamp = std::stoull(timestring);
            std::string camname = strs.at(0);

            if(boost::iequals(camname,"bottom") && useBottom_){
              bottomImageNames.push_back(std::make_pair(timestamp, img.path().filename().string()));
            }
            else if(boost::iequals(camname,"front") && useFront_){
              frontImageNames.push_back(std::make_pair(timestamp, img.path().filename().string()));
            }
            else if(boost::iequals(camname,"left") && useLeft_){
              leftImageNames.push_back(std::make_pair(timestamp, img.path().filename().string()));
            }
            else if(boost::iequals(camname,"right") && useRight_){
              rightImageNames.push_back(std::make_pair(timestamp, img.path().filename().string()));
            }
            else if(boost::iequals(camname,"top") && useTop_){
              topImageNames.push_back(std::make_pair(timestamp, img.path().filename().string()));
            }
          }
          // sort vectors
          std::sort(bottomImageNames.begin(),bottomImageNames.end()); // should per default sort by first element
          std::sort(frontImageNames.begin(),frontImageNames.end()); // should per default sort by first element
          std::sort(leftImageNames.begin(),leftImageNames.end()); // should per default sort by first element
          std::sort(rightImageNames.begin(),rightImageNames.end()); // should per default sort by first element
          std::sort(topImageNames.begin(),topImageNames.end()); // should per default sort by first element

          if(useBottom_){
            allImageNames_.push_back(bottomImageNames);
            LOG(INFO)<< "bottom number of images: " << bottomImageNames.size();
          }
          if(useFront_){
            allImageNames_.push_back(frontImageNames);
            LOG(INFO)<< "front number of images: " << frontImageNames.size();
          }
          if(useLeft_){
            allImageNames_.push_back(leftImageNames);
            LOG(INFO)<< "left number of images: " << leftImageNames.size();
          }
          if(useRight_){
            allImageNames_.push_back(rightImageNames);
            LOG(INFO)<< "right number of images: " << rightImageNames.size();
          }
          if(useTop_){
            allImageNames_.push_back(topImageNames);
            LOG(INFO)<< "top number of images: " << topImageNames.size();
          }
        }
      }

      counter_ = 0;
      numImages_ = allImageNames_[0].size();
      streaming_ = true;
      processingThread_ = std::thread(&LeicaDatasetReader::processing, this);

      return true;
    }

    bool LeicaDatasetReader::stopStreaming() {
      // Stop the pipeline
      if(processingThread_.joinable()) {
        processingThread_.join();
        streaming_ = false;
      }
      return true;
    }

    void  LeicaDatasetReader::processing() {

      std::string line;
      okvis::Time start(0.0);
      const size_t numCameras = allImageNames_.size();
      std::vector < std::vector < std::pair<uint64_t, std::string> > ::iterator
      > cam_iterators(numCameras);
      for (size_t i = 0; i < numCameras; ++i) {
        cam_iterators.at(i) = allImageNames_.at(i).begin();
      }

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
            uint64_t tcam = cam_iterators.at(i)->first;//std::atol(cam_iterators.at(i)->first.c_str());
            if(tcam>max_timestamp) {
              max_timestamp = tcam;
            }
          }
          synched=true;
          for (size_t i = 0; i < numCameras; ++i) {
            uint64_t tcam = cam_iterators.at(i)->first;//std::atol(cam_iterators.at(i)->first.c_str());
            if(tcam < max_timestamp - tolNSec) {
              cam_iterators.at(i)++; // not in tolerance, advance
              synched = false;
            }
          }
        }

        // check synched (I am paranoid)
        for (size_t i = 0; i < numCameras; ++i) {
          if(i>0) {
            int64_t tcam = cam_iterators.at(0)->first;//std::atol(cam_iterators.at(0)->first.c_str());
            int64_t tcam_i = cam_iterators.at(i)->first;//std::atol(cam_iterators.at(i)->first.c_str());
            OKVIS_ASSERT_TRUE(Exception, abs(tcam-tcam_i)<int64_t(tolNSec),
                              "timestamp mismatch " << double(abs(tcam-tcam_i))/1.0e6 << " ms at " << tcam_i);
          }
        }

        // add images
        okvis::Time t;
        std::vector<cv::Mat> images(numCameras);
        for (size_t i = 0; i < numCameras; ++i) {
          std::string filename;
          if(camType_ == "pinhole")
            filename = path_ + "/pinhole/" + cam_iterators.at(i)->second;
          else if(camType_ == "eucm")
            filename = path_ + "/imgs/" + cam_iterators.at(i)->second;
          //std::cout << "opening image " << filename << std::endl;
          cv::Mat filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE);
          //if(i==0){
          //cv::imshow("test", filtered);
          //cv::waitKey(0);
          //}

          OKVIS_ASSERT_TRUE(
                  Exception, !filtered.empty(),
                  "cam " << i << " missing image :" << std::endl << filename)

          //t.fromNSec(std::atol(cam_iterators.at(i)->first.c_str()));
          t.fromNSec(cam_iterators.at(i)->first);

          if (start == okvis::Time(0.0)) {
            start = t;
          }

          // get all LiDAR measurements till then
          okvis::Time t_lidar = start;
          std::string lidarLine;
          do {
            if (!std::getline(lidarFile_, lidarLine)) {
              streaming_ = false;
              return;
            }

            std::stringstream lidarStream(lidarLine);
            std::string sl;
            // 1st entry timestamp
            std::getline(lidarStream, sl, ',');
            uint64_t lidarNanoseconds = std::stol(sl.c_str());

            // 2nd - 4th entry: xyz coordinates in LiDAR frame
            Eigen::Vector3d ptXYZ;
            for(int j = 0; j < 3; ++j){
              std::getline(lidarStream, sl, ',');
              ptXYZ[j] = std::stod(sl);
            }
            // 5th entry: intensity
            std::getline(lidarStream, sl);

            t_lidar.fromNSec(lidarNanoseconds);

            // add the lidar measurement for (blocking) processing
            if (t_lidar - start + okvis::Duration(1.0) > deltaT_) {
              lidarCallback_(t_lidar, ptXYZ);
            }

          } while (t_lidar <= t);

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

            Eigen::Vector3d acc;
            for (int j = 0; j < 3; ++j) {
              std::getline(stream, s, ',');
              acc[j] = std::stof(s);
            }

            Eigen::Vector3d gyr;
            for (int j = 0; j < 3; ++j) {
              std::getline(stream, s, ',');
              gyr[j] = std::stof(s);
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
                  /*std::cout << "geodetic leica callback at " << t_gps_.nsec
                            << " with lat | lon | alt | hErr | vErr: "
                            << lat << " | " << lon << " | " << alt << " | "
                            << hErr << " | " << vErr << std::endl;*/
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
      }

      return;
    }

    bool LeicaDatasetReader::readLidarData(std::vector<okvis::Time>& timestamps, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& lidarMeasurements){
      std::string lidarFileName = path_ + "/lidar.csv";
      // Open the Lidar File
      std::ifstream lidarFile_;
      lidarFile_.open(lidarFileName);
      if(!lidarFile_.good()){
        LOG(ERROR) << "No LiDAR file found at " << lidarFileName << std::endl;
        return false;
      }
      // Iterate through whole file
      std::string line;
      int number_of_lines = 0;
      while (std::getline(lidarFile_, line))
        ++number_of_lines;

      LOG(INFO) << "No. LiDAR measurements: " << number_of_lines-1 << std::endl;
      if (number_of_lines - 1 <= 0) {
        LOG(ERROR) << "no LiDAR messages present in " << lidarFileName << std::endl;
        return false;
      }

      // set reading position to second line
      lidarFile_.clear();
      lidarFile_.seekg(0, std::ios::beg);
      std::getline(lidarFile_, line);

      // Now read all LiDAR measurements
      uint64_t ts;
      int intensity;
      Eigen::Vector3d ptXYZ;

      while(std::getline(lidarFile_, line)){

        // Parse single line
        std::stringstream stream(line);
        std::string s;

        // 1st entry: timestamp
        std::getline(stream, s, ',');
        ts = std::stol(s.c_str());

        // 2nd - 4th entry: xyz coordinates in LiDAR frame
        for(int j = 0; j < 3; ++j){
          std::getline(stream, s, ',');
          ptXYZ[j] = std::stod(s);
        }
        // save measurement
        okvis::Time time;
        time.fromNSec(ts);
        timestamps.push_back(time);
        lidarMeasurements.push_back(ptXYZ);

        // 5th entry: intensity
        std::getline(stream, s);
        intensity = std::stoi(s);
      }
      return true;
    }
}
