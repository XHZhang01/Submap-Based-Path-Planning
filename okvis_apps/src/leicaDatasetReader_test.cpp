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
 *  Created on: Jun 26, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *              Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file leicaDatasetReader_test.cpp
 * @brief This file tests processing a Leica format dataset.
 * @author Simon Boche
 * */

#include <iostream>
#include <stdlib.h>
#include <memory>
#include <functional>

#include <Eigen/Core>

#include <opencv2/highgui/highgui.hpp>

#include <okvis/DatasetReader.hpp>
#include <okvis/LeicaDatasetReader.hpp>
#include <okvis/ViParametersReader.hpp>
#include <okvis/ThreadedSlam.hpp>
#include <okvis/TrajectoryOutput.hpp>
#include <boost/filesystem.hpp>

const okvis::kinematics::Transformation T_SL(Eigen::Vector3d(-0.071908144960453, -0.002367340978787, 0.032844126653258),
                                             Eigen::Quaterniond(0.704215721901235, 0.000900815612216, -0.709709873853548, 0.019781317275051));


// this is just a workbench. most of the stuff here will go into the Frontend class.
int main(int argc, char **argv)
{
  // argv [1] --> okvis config
  // argv [2] --> dataset path

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  /*if (argc != 3) {
    LOG(ERROR)<<
              "Usage: ./" << argv[0] << " configuration-yaml-file dataset-folder";
    return EXIT_FAILURE;
  }*/

  okvis::Duration deltaT(0.0);

  // read configuration file
  std::string configFilename(argv[1]);

  okvis::ViParametersReader viParametersReader(configFilename);
  okvis::ViParameters parameters;
  viParametersReader.getParameters(parameters);

  // dataset reader
  std::string path(argv[2]);
  std::shared_ptr<okvis::LeicaDatasetReader> datasetReader;

  if(parameters.nCameraSystem.cameraGeometry(0)->type() == "EUCMCamera")
    datasetReader.reset(new okvis::LeicaDatasetReader(path,deltaT, true, true, true, true, false, "eucm", parameters.gps));
  else if(parameters.nCameraSystem.cameraGeometry(0)->type() == "PinholeCamera")
    datasetReader.reset(new okvis::LeicaDatasetReader(path,deltaT, true, true, true, true, false, "pinhole", parameters.gps));
  else{
    LOG(ERROR) << "Unknown camera type.";
    return EXIT_FAILURE;
  }

  // also check DBoW2 vocabulary
  boost::filesystem::path executable(argv[0]);
  std::string dBowVocDir = executable.remove_filename().string();
  std::ifstream infile(dBowVocDir+"/small_voc.yml.gz");
  if(!infile.good()) {
    LOG(ERROR)<<"DBoW2 vocaublary " << dBowVocDir << "/small_voc.yml.gz not found.";
    return EXIT_FAILURE;
  }
  LOG(INFO) << "PROCESSED THE DBOW";
  okvis::ThreadedSlam estimator(parameters, dBowVocDir);
  estimator.setBlocking(true);

  // write logs
  std::string mode = "slam";
  if(!parameters.estimator.do_loop_closures) {
    mode = "vio";
  }
  if(parameters.camera.online_calibration.do_extrinsics) {
    mode = mode+"-calib";
  }


  okvis::TrajectoryOutput writer(path+"/okvis2-" + mode + "_trajectory.csv", false);
  estimator.setOptimisedGraphCallback(
          std::bind(&okvis::TrajectoryOutput::processState, &writer,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4));
  estimator.setFinalTrajectoryCsvFile(path+"/okvis2-" + mode + "-final_trajectory.csv");
  estimator.setMapCsvFile(path+"/okvis2-" + mode + "-final_map.csv");

  // connect reader to estimator
  datasetReader->setImuCallback(
          std::bind(&okvis::ThreadedSlam::addImuMeasurement, &estimator,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  datasetReader->setImagesCallback(
          std::bind(&okvis::ThreadedSlam::addImages, &estimator, std::placeholders::_1,
                    std::placeholders::_2));
  datasetReader->setLidarCallback(
          std::bind(&okvis::ThreadedSlam::addLidarMeasurement, &estimator, std::placeholders::_1, std::placeholders::_2));

  if(parameters.gps.use_gps){

    if(parameters.gps.type == "cartesian"){
      datasetReader->setGpsCallback(
              std::bind(&okvis::ThreadedSlam::addGpsMeasurement, &estimator,
                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
    else if(parameters.gps.type == "geodetic" || parameters.gps.type == "geodetic-leica"){
      datasetReader->setGeodeticGpsCallback(
              std::bind(&okvis::ThreadedSlam::addGeodeticGpsMeasurement, &estimator,
                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                        std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
    }
    else{
      LOG(ERROR) << "Unknown GPS data type.";
      return EXIT_FAILURE;
    }

  }


  //datasetReader->setLidarCallback(
  //        std::bind(&okvis::ThreadedSlam::addLidarMeasurement, &estimator, std::placeholders::_1,
  //                  std::placeholders::_2));

  // start
  okvis::Time startTime = okvis::Time::now();
  datasetReader->startStreaming();
  int progress = 0;
  while (true) {
    //LOG(INFO) << "Start processing the frames";

    estimator.processFrame();
    cv::Mat matches, overhead;
    estimator.display(matches, overhead);
    if(!matches.empty()){
      cv::imshow("OKVIS2 Matches", matches);
    }
    cv::Mat topView;
    writer.drawTopView(topView);
    cv::imshow("OKVIS2 Top View", topView);
    cv::waitKey(2);

    // check if done
    if(!datasetReader->isStreaming()) {
      std::cout << "\rFinished!" << std::endl;
      if(parameters.estimator.do_final_ba) {
        LOG(INFO) << "final full BA...";
        estimator.doFinalBa();
        cv::waitKey(1);
      }
      LOG(INFO) <<"total processing time OKVIS only " << (okvis::Time::now() - startTime) << " s" << std::endl;
      estimator.writeFinalTrajectoryCsv();
      estimator.writeGlobalTrajectoryCsv(path+"/okvis2-" + mode + "-global-final_trajectory.csv");
      estimator.saveMap();
      //okvis::Time mappingStartTime = okvis::Time::now();
      //estimator.createFinalSe2Map();
      //LOG(INFO) <<"total processing time for map " << (okvis::Time::now() - mappingStartTime) << " s" << std::endl;
      //estimator.exportSe2Map(); // ToDo: test switching on  joining for mesh export
      LOG(INFO) <<"total processing time " << (okvis::Time::now() - startTime) << " s" << std::endl;
      cv::waitKey(1);
      break;
    }

    // display progress
    int newProgress = int(datasetReader->completion()*100.0);
#ifndef DEACTIVATE_TIMERS
    if (newProgress>progress) {
      LOG(INFO) << okvis::timing::Timing::print();
    }
#endif
    if (newProgress>progress) {
      progress = newProgress;
      std::cout << "\rProgress: "
                << progress << "% "
                << std::flush;
    }
  }

  // read lidar data
  std::vector<okvis::Time> lidarTimestamps;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lidarMeasurements;
  //datasetReader->readLidarData(lidarTimestamps, lidarMeasurements);

  // Visualise LiDAR point cloud
  //writer.visualiseLidarData(T_SL, lidarTimestamps, lidarMeasurements);

  return EXIT_SUCCESS;
}
