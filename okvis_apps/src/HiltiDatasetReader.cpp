//
// Created by boche on 2/23/23.
//

/**
 * @file HiltiDatasetReader.cpp
 * @brief Source file for the HiltiDatasetReader class. 99% taken of DatasetReader.cpp
 * @author Simon Boche
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <okvis/HiltiDatasetReader.hpp>

namespace okvis {

HiltiDatasetReader::HiltiDatasetReader(const std::string & path,
                             const Duration & deltaT, int numCameras, GpsParameters gpsParameters, bool use_lidar) :
        deltaT_(deltaT), numCameras_(numCameras), gpsFlag_(gpsParameters.use_gps), gpsDataType_(gpsParameters.type),
        lidarFlag_(use_lidar){
  OKVIS_ASSERT_TRUE(Exception, gpsDataType_=="cartesian" || gpsDataType_=="geodetic" || gpsDataType_=="geodetic-leica",
                    "Unknown GPS data type specified")
  streaming_ = false;
  setDatasetPath(path);
  counter_ = 0;
  t_gps_ = okvis::Time(0.0);
}

HiltiDatasetReader::~HiltiDatasetReader() {
  stopStreaming();
}

bool HiltiDatasetReader::setDatasetPath(const std::string & path) {
  path_ = path;
  return true;
}

bool HiltiDatasetReader::setStartingDelay(const Duration &deltaT)
{
  if(streaming_) {
    LOG(WARNING)<< "starting delay ignored, because streaming already started";
    return false;
  }
  deltaT_ = deltaT;
  return true;
}

bool HiltiDatasetReader::isStreaming()
{
  return streaming_;
}

double HiltiDatasetReader::completion() const {
  if(streaming_) {
    return double(counter_)/double(numImages_);
  }
  return 0.0;
}

bool HiltiDatasetReader::startStreaming() {
  OKVIS_ASSERT_TRUE(Exception, imagesCallback_, "no add image callback registered")
  OKVIS_ASSERT_TRUE(Exception, imuCallback_, "no add IMU callback registered")
  if(gpsFlag_)
    OKVIS_ASSERT_TRUE(Exception, geodeticGpsCallback_ || gpsCallback_, "no add GPS callback registered")
  if(lidarFlag_)
    OKVIS_ASSERT_TRUE(Exception, lidarCallback_, "no add Lidar callback registered")

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

  if(lidarFlag_){
    // Open LiDAR file
    std::string lidarLine;
    lidarFile_.open(path_ + "/lidar0/data.csv");
    OKVIS_ASSERT_TRUE(Exception, lidarFile_.good(), "no lidar file found at " << path_+"/lidar0/data.csv");
    int number_of_lidar_lines = 0;
    while (std::getline(lidarFile_, lidarLine))
      ++number_of_lidar_lines;
    LOG(INFO)<< "No. LiDAR measurements: " << number_of_lidar_lines-1;
    if (number_of_lidar_lines - 1 <= 0) {
      LOG(ERROR)<< "no lidar messages present in " << path_+"/lidar0/data.csv";
      return -1;
    }
    // set reading position to second line
    lidarFile_.clear();
    lidarFile_.seekg(0, std::ios::beg);
    std::getline(lidarFile_, lidarLine);
  }

  // now open camera files
  std::vector<okvis::Time> times;
  okvis::Time latest(0);
  int num_camera_images = 0;
  int i = 0;
  while (i<numCameras_) {
    std::ifstream camDataFile(path_ + "/cam" + std::to_string(i) + "/data.csv");
    if(!camDataFile.good()) {
      OKVIS_ASSERT_TRUE(Exception, i>0, "No camera data found");
      break;
    }
    num_camera_images = 0;
    std::vector < std::pair<std::string, std::string> > imageNames;
    std::getline(camDataFile, line);
    while (std::getline(camDataFile, line)) {
      ++num_camera_images;
      std::stringstream stream(line);
      std::string s0, s1;
      std::getline(stream, s0, ',');
      imageNames.push_back(std::make_pair(s0,s0+".png"));
    }
    allImageNames_.push_back(imageNames);
    LOG(INFO)<< "No. cam " << i << " images: " << num_camera_images;
    if(i==0) {
      numImages_ = num_camera_images;
    }
    ++i; // try next camera
  }

  counter_ = 0;
  streaming_ = true;
  processingThread_ = std::thread(&HiltiDatasetReader::processing, this);

  return true;
}

bool HiltiDatasetReader::stopStreaming() {
  // Stop the pipeline
  if(processingThread_.joinable()) {
    processingThread_.join();
    streaming_ = false;
  }
  return true;
}

void  HiltiDatasetReader::processing() {
  std::string line;
  okvis::Time start(0.0);
  const size_t numCameras = allImageNames_.size();
  std::vector < std::vector < std::pair<std::string, std::string> > ::iterator
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

      if(lidarFlag_){
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
  }

  return;
}

}
