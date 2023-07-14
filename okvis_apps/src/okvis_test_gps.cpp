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
 * @file okvis_test_gps.cpp
 * @brief
 * @author Simon Boche
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>
#include <stdlib.h>
#include <unistd.h>
#include <random>

#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#pragma GCC diagnostic pop
#include <okvis/ViParametersReader.hpp>
#include <okvis/ThreadedSlam.hpp>
#include <okvis/DatasetReader.hpp>
#include <okvis/RpgDatasetReader.hpp>
#include <okvis/TrajectoryOutput.hpp>
#include <boost/filesystem.hpp>

// Function visualising GPS signal
void createVisualisation(cv::Mat &img , std::vector<cv::Point3d> trajectory, std::vector<cv::Point3d> measurements){
    int imgWidth = img.size().width;
    int imgHeight = img.size().height;


    // find minimum / maximum coordinates

    double x_min,x_max, y_min,y_max;
    x_min = trajectory.front().x;
    x_max = trajectory.front().x;
    y_min = trajectory.front().y;
    y_max = trajectory.front().y;

    for(auto iter = trajectory.begin() ; iter!= trajectory.end() ; iter++){
        if(iter->x < x_min)
            x_min = iter->x;
        if(iter->x > x_max)
            x_max = iter->x;
        if(iter->y < y_min)
            y_min = iter->y;
        if(iter->y > y_max)
            y_max = iter->y;
    }

    // now iterate points, scale them and draw line
    cv::Point2d prevPoint(0.0,0.0);
    cv::Point2d currPoint(0.0,0.0);
    cv::Point2d currMeas(0.0,0.0);
    int count = 0;

    for(auto iter = trajectory.begin() ; iter!= trajectory.end() ; iter++){
        currPoint.x = (iter->x - x_min) / (x_max-x_min) * imgWidth;
        currPoint.y = (iter->y - y_min) / (y_max-y_min) * imgHeight;
        if(count>0)
            cv::line(img, prevPoint,currPoint,cv::Scalar(255,0,0));
        count+=1;
        prevPoint = currPoint;
    }

    for(auto iter = measurements.begin() ; iter != measurements.end() ; iter++){
        currMeas.x = (iter->x - x_min) / (x_max-x_min) * imgWidth;
        currMeas.y = (iter->y - y_min) / (y_max-y_min) * imgHeight;
        cv::circle(img, currMeas, 1.0, cv::Scalar(0,0,255));
    }

}

// Function creating GPS data on the fly
void createGpsData(std::string dataSetPath, double uncertaintyParameter, int nGpsDropouts, int frameDelta, Eigen::Vector3d r_SA, bool tumvi){

    std::string GT_FILE ="";

    if(tumvi)
      GT_FILE = dataSetPath + "mocap0/data.csv";
    else
      GT_FILE = dataSetPath + "state_groundtruth_estimate0/data.csv";
    std::cout << GT_FILE << std::endl;
    const std::string OUT_FOLDER = dataSetPath + "gps0/";

    // ----- Visualization stuff initialization - begin -----
    cv::Mat visualization(800, 800, CV_8UC3);
    std::vector<cv::Point3d> fullTrajectoryPoints;
    std::vector<cv::Point3d> gpsPositionMeasurements;
    // ----- Visualization stuff initialization - end   -----

    // Initialize random number generators
    std::default_random_engine genx;
    genx.seed(1);
    std::normal_distribution<double> distributionx(0.0, uncertaintyParameter);
    std::default_random_engine geny;
    geny.seed(2);
    std::normal_distribution<double> distributiony(0.0, uncertaintyParameter);
    std::default_random_engine genz;
    genz.seed(3);
    std::normal_distribution<double> distributionz(0.0, uncertaintyParameter);


    // Read in GPS signal (from Ground-Truth Data)
    std::ifstream gpsSignal;
    // open the GPS file
    std::string gline;
    gpsSignal.open(GT_FILE);
    int gnumber_of_lines = 0;
    while (std::getline(gpsSignal, gline))
      ++gnumber_of_lines;
    std::cout << "No. GPS measurements: " << gnumber_of_lines-1 << std::endl;
    if (gnumber_of_lines - 1 <= 0) {
      std::cout << "ERROR: no gps messages present" << std::endl;
    }

    // prepare dropout of gps signals
    int nIntervals = 2*nGpsDropouts + 1;
    double intervalLengthRelative = 1.0 / (static_cast<double>(nIntervals) );
    size_t signalCounter = 0; // counter to determine when GPS loss will be registered

    // set reading position to second line
    gpsSignal.clear();
    gpsSignal.seekg(0, std::ios::beg);
    std::getline(gpsSignal, gline);


    // output file name
    std::string outputFileName = OUT_FOLDER + "data.csv";
    std::ofstream outputFile(outputFileName);
    outputFile << "#timestamp , p_x , p_y , p_z , sigma_x , sigma_y , sigma_z \n";

    // placeholder variables for read variables
    okvis::Time tg;
    std::string timestring;
    int count = 0; // counter for gps signal
    double px, py, pz;

    std::string line;
    while(std::getline(gpsSignal,line)){
        signalCounter += 1;
        std::stringstream stream(line);
        std::string s;
        std::getline(stream, s, ',');
        if(s[0] == '#')
            continue;
        timestring = s;

        // check for frame deltas
        if(count!=frameDelta){
          count+=1;
          continue;
        }
        else{
          count = 0;
        }

        Eigen::Vector3d pos;
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ',');
          pos[j] = std::stof(s);
        }

        // Ground-Truth Orientation
        Eigen::Quaterniond ori;
        std::getline(stream, s, ',');
        ori.w() = std::stof(s);
        std::getline(stream, s, ',');
        ori.x() = std::stof(s);
        std::getline(stream, s, ',');
        ori.y() = std::stof(s);
        std::getline(stream, s, ',');
        ori.z() = std::stof(s);

        // Apply r_SA (antenna position relative to sensor)
        okvis::kinematics::Transformation T_WS(pos,ori);
        pos += T_WS.C()*r_SA;

        // Disturb GT position to create GPS Signal
        double sigx = distributionx(genx);
        px = pos[0] + sigx;
        double sigy = distributiony(geny);
        py = pos[1] + sigy;
        double sigz = distributionz(genz);
        pz = pos[2] + sigz;

        // Save gt position for trajectory
        fullTrajectoryPoints.push_back(cv::Point3d(pos[0] , pos[1] , pos[2]));

        double progress = static_cast<double>(signalCounter)/ static_cast<double>(gnumber_of_lines-1);

        double intervalRemainder = progress / intervalLengthRelative;

        if(static_cast<int>(std::floor(intervalRemainder)) % 2 == 0){

            gpsPositionMeasurements.push_back(cv::Point3d(px , py , pz));
            // write to output file
            outputFile << timestring << ","
                       << px << ","
                       << py << ","
                       << pz << ","
                       << uncertaintyParameter << ","
                       << uncertaintyParameter << ","
                       << uncertaintyParameter << std::endl;


          }


//        if(count == frameDelta){
        //count+=1;

//            // Save gt position for trajectory
//            fullTrajectoryPoints.push_back(cv::Point3d(pos[0] , pos[1] , pos[2]));
//            if(progress < gpsLoss || (progress > gpsReturn && progress < gpsLoss2) || (progress > gpsReturn2)){
//                gpsPositionMeasurements.push_back(cv::Point3d(px , py , pz));
//                // write to output file
//                outputFile << timestring << ","
//                           << px << ","
//                           << py << ","
//                           << pz << ","
//                           << uncertaintyParameter << ","
//                           << uncertaintyParameter << ","
//                           << uncertaintyParameter << std::endl;
//            }

//            count = 0;

//        }


    }

    outputFile.close();

    std::cout << "Created " << gpsPositionMeasurements.size() << " GPS measurements subsampled from " << fullTrajectoryPoints.size() << " trajectory points." << std::endl;
    std::cout << "signalCounter: "<< signalCounter << std::endl;
    std::cout << "gnumber_oof_lines" << gnumber_of_lines << std::endl;
    createVisualisation(visualization, fullTrajectoryPoints, gpsPositionMeasurements);
    cv::imshow("TrajectoryVis", visualization);
    //cv::waitKey(0);
}

// this is just a workbench. most of the stuff here will go into the Frontend class.
int main(int argc, char **argv)
{

    // ----- GPS Test parameters - Begin
    // Eigen::Vector3d r_SA(0.15, 0.1, 0.05);
    // double observabilityThresh = 5.0; 5 for TUM VI, 20 for EUROC MAV (for low gps errors
    double gpsNoiseParameter = 0.0;
    // ----- GPS Test parameters - End

    // Main Processing - START

    google::InitGoogleLogging(".");
    FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1;

    if (argc != 4 && argc != 7) {
      LOG(ERROR)<<
      "Usage: ./" << argv[0] << " configuration-yaml-file dataset-folder [-euroc / -tumvi] ([-gps] noiseparameter n_dropouts)";
      //
      return EXIT_FAILURE;
    }

    okvis::Duration deltaT(0.0);

    // read configuration file
    std::string configFilename(argv[1]);

    okvis::ViParametersReader viParametersReader(configFilename);
    okvis::ViParameters parameters;
    viParametersReader.getParameters(parameters);

    // read gps parameters from command line (if given)
    bool tumvi = false;
    if(strcmp(argv[3], "-tumvi")==0)
      tumvi = true;
    else{
        if(strcmp(argv[3], "-euroc")!=0)
          return EXIT_FAILURE;
      }

    int framedelta;
    if (tumvi)
      framedelta = 0;
    else
      framedelta = 0;

    bool useGps = false;
    int nGpsDropouts;
    if (argc > 4) {
      if(strcmp(argv[4], "-gps")==0) {
        useGps = true;
      }
      gpsNoiseParameter = atof(argv[5]);
      nGpsDropouts = atoi(argv[6]);
      std::cout << "Running simulation with GPS data of gaussian random noise with sigma = " << gpsNoiseParameter << " and " << nGpsDropouts << " dropouts." << std::endl;
    }
    else{
        std::cout << "Running simulation without GPS data." << std::endl;
    }

    // dataset reader
    // the folder path
    std::string path(argv[2]);
    std::shared_ptr<okvis::DatasetReaderBase> datasetReader;
    // create gps data if necessary
    if(useGps)
        createGpsData(path, gpsNoiseParameter, nGpsDropouts, framedelta, parameters.gps.r_SA, tumvi); // 0.3, 0.5, 0.505, 0.8 // FrameDelta 1 for TUM VI, 8 for EUROC MAV ; gps loss return for EUROC MAV 0.4 0.8
    datasetReader.reset(new okvis::DatasetReader(path, deltaT, parameters.nCameraSystem.numCameras(), parameters.gps /* -> set if we want to use gps data*/));

    // also check DBoW2 vocabulary
    boost::filesystem::path executable(argv[0]);
    std::string dBowVocDir = executable.remove_filename().string();
    std::ifstream infile(dBowVocDir+"/small_voc.yml.gz");
    if(!infile.good()) {
       LOG(ERROR)<<"DBoW2 vocaublary " << dBowVocDir << "/small_voc.yml.gz not found.";
       return EXIT_FAILURE;
    }

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
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4));
  estimator.setFinalTrajectoryCsvFile(path+"/okvis2-" + mode + "-final_trajectory.csv");
  estimator.setMapCsvFile(path+"/okvis2-" + mode + "-final_map.csv");

    // connect reader to estimator
    datasetReader->setImuCallback(
          std::bind(&okvis::ThreadedSlam::addImuMeasurement, &estimator,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    datasetReader->setImagesCallback(
          std::bind(&okvis::ThreadedSlam::addImages, &estimator, std::placeholders::_1,
                    std::placeholders::_2));

    if(parameters.gps.use_gps){
        datasetReader->setGpsCallback(
              std::bind(&okvis::ThreadedSlam::addGpsMeasurement, &estimator,
                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    // start
    okvis::Time startTime = okvis::Time::now();
    datasetReader->startStreaming();
    int progress = 0;
    while (true) {
      estimator.processFrame();
      estimator.display();
      writer.drawTopView();
      cv::waitKey(2);

      // check if done
      if(!datasetReader->isStreaming()) {
        std::cout << "\rFinished!" << std::endl;
        if(parameters.estimator.do_final_ba) {
          LOG(INFO) << "final full BA..." << std::endl;
          estimator.doFinalBa();
          cv::waitKey();
        }
        estimator.writeFinalTrajectoryCsv();
        estimator.writeGlobalTrajectoryCsv(path+"/okvis2-" + mode + "-global_trajectory.csv");
        //estimator.saveMap();
        LOG(INFO) <<"total processing time " << (okvis::Time::now() - startTime) << " s" << std::endl;
        cv::waitKey(1000);
        // evaluate final gps residuals
//        if(useGps)
//            estimator.dumpGpsResiduals(path+"/okvis2-gpsResiduals.csv");
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

    return EXIT_SUCCESS;
}
