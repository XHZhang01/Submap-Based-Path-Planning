//
// Created by boche on 2/24/23.
//
/**
 * @file okvis_app_hilti_submaps.cpp
 * @brief This file processes a HILTI dataset including submapping (ray-based).
 * @author Simon Boche
 * */

#include <iostream>
#include <stdlib.h>
#include <memory>
#include <functional>

#include <Eigen/Core>

// okvis & supereightinterface
#include <okvis/HiltiDatasetReader.hpp>
#include <okvis/ViParametersReader.hpp>
#include <okvis/ThreadedSlam.hpp>
#include <okvis/TrajectoryOutput.hpp>
#include <boost/filesystem.hpp>
#include <okvis/SubmappingInterface.hpp>
#include <utility>

class SubmapInterfacer{

private:

    // ============= OKVIS + SE =============

    // Dataset reader
    std::shared_ptr<okvis::HiltiDatasetReader> datasetReader;

    // okvis interface
    std::shared_ptr<okvis::ThreadedSlam> okvis_estimator;

    // supereight interface
    std::shared_ptr<okvis::SubmappingInterface> se_interface;

    // to write traj estimate
    std::shared_ptr<okvis::TrajectoryOutput> writer;

    // to configure okvis
    okvis::ViParameters parameters;
    std::string path;
    std::string mode;

    // check if integration is running
    bool isIntegrating_ = false;

    // ============= SOME PVT FUNCTIONS =============

    void slam_loop();

    bool isIntegrating(){ return isIntegrating_; }

    // ============= THREADS =============

    //std::thread thread_okvis;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SubmapInterfacer() = delete;

    SubmapInterfacer(char** argv);

    ~SubmapInterfacer();

    // Starts the processing loop
    int start();

};

SubmapInterfacer::SubmapInterfacer(char **argv) {

  // argv [1] --> okvis config
  // argv [2] --> dataset path
  // argv [3] --> mapping config file

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  okvis::Duration deltaT(0.0);

  // read configuration file
  std::string configFilename(argv[1]);
  std::string submappingConfigFilename(argv[3]);

  okvis::ViParametersReader viParametersReader(configFilename);
  viParametersReader.getParameters(parameters);

  // dataset reader
  path = std::string(argv[2]);
  datasetReader.reset(new okvis::HiltiDatasetReader(path, deltaT, parameters.nCameraSystem.numCameras(),
                                                    parameters.gps, true));

  // also check DBoW2 vocabulary
  boost::filesystem::path executable(argv[0]);
  std::string dBowVocDir = executable.remove_filename().string();
  std::ifstream infile(dBowVocDir+"/small_voc.yml.gz");
  assert(infile.good());

  // Setup OKVIS estimatpr
  //std::shared_ptr<okvis::ThreadedSlam> estimator;
  okvis_estimator.reset(new okvis::ThreadedSlam(parameters,dBowVocDir));// = std::make_shared<okvis::ThreadedSlam>(parameters, dBowVocDir);
  okvis_estimator->setBlocking(true);

  // Setup SE2 Submapping Interface
  const Eigen::Matrix4d T_SC = parameters.nCameraSystem.T_SC(0)->T();
  se::OkvisSubmapsConfig<se::OccupancyDataConfig, se::LeicaLidarConfig> se_config(submappingConfigFilename);
  std::cout << "Created LiDAR Mapping backend with following parameters:\n" << se_config << std::endl;
  const se::MapConfig mapConfig(se_config.se_config.map);
  const se::OccupancyDataConfig dataConfig(se_config.se_config.data);
  se::LeicaLidarConfig sensorConfig(se_config.se_config.sensor);
  float timeThresholdSec = se_config.general.submapTimeThreshold;
  okvis::Duration timeThreshold(floor(timeThresholdSec),0);
  se::LeicaLidar lidar(sensorConfig);

  std::pair<Eigen::Matrix4d, se::LeicaLidar> lidarSensor = std::make_pair(sensorConfig.T_BS.cast<double>(), lidar);

  se_interface.reset(new okvis::SubmappingInterface(mapConfig, dataConfig, se_config.general, &lidarSensor));
  se_interface->setBlocking(true);

  // write logs
  //std::string mode = "slam";
  mode = "slam";
  if(!parameters.estimator.do_loop_closures) {
    mode = "vio";
  }
  if(parameters.camera.online_calibration.do_extrinsics) {
    mode = mode+"-calib";
  }

  writer.reset(new okvis::TrajectoryOutput(path+"/okvis2-" + mode + "_trajectory.csv", false)); // = std::make_shared<okvis::TrajectoryOutput>(path+"/okvis2-" + mode + "_trajectory.csv", false);
  okvis_estimator->setOptimisedGraphCallback([&] (const okvis::State& _1, const okvis::TrackingState& _2,
                                                  std::shared_ptr<const okvis::AlignedMap<StateId, State>> _3,
                                                  std::shared_ptr<const okvis::MapPointVector> _4){
      writer->processState(_1,_2,_3,_4);
      se_interface->stateUpdateCallback(_1,_2,_3);
  });
  okvis_estimator->setFinalTrajectoryCsvFile(path+"/okvis2-" + mode + "-final_trajectory.csv");
  okvis_estimator->setMapCsvFile(path+"/okvis2-" + mode + "-final_map.csv");

  se_interface->setAlignCallback(std::bind(&okvis::ThreadedSlam::addSubmapAlignmentConstraints, okvis_estimator,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                           std::placeholders::_4));

  // connect reader to estimator
  datasetReader->setImuCallback(
          std::bind(&okvis::ThreadedSlam::addImuMeasurement, okvis_estimator,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  datasetReader->setImagesCallback(
          std::bind(&okvis::ThreadedSlam::addImages, okvis_estimator, std::placeholders::_1,
                    std::placeholders::_2));

  assert(parameters.gps.type == "cartesian" || parameters.gps.type == "geodetic" || parameters.gps.type == "geodetic-leica");
  if(parameters.gps.use_gps){
    if(parameters.gps.type == "cartesian"){
      datasetReader->setGpsCallback(
              std::bind(&okvis::ThreadedSlam::addGpsMeasurement, okvis_estimator,
                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
    else if(parameters.gps.type == "geodetic" || parameters.gps.type == "geodetic-leica"){
      datasetReader->setGeodeticGpsCallback(
              std::bind(&okvis::ThreadedSlam::addGeodeticGpsMeasurement, okvis_estimator,
                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                        std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
    }
  }
  datasetReader->setLidarCallback(
          std::bind(&okvis::SubmappingInterface::addLidarMeasurement, se_interface,
                    std::placeholders::_1, std::placeholders::_2));

}

SubmapInterfacer::~SubmapInterfacer()
{

}


int SubmapInterfacer::start()
{

  // =============== START THINGS UP ===============

  // Start supereight processing
  se_interface->start(); // ToDo: not really needed

  // Start datasetreader streaming
  datasetReader->startStreaming();

  // SLAM & Mapping Loop
  slam_loop();

  return 0;

}

void SubmapInterfacer::slam_loop(){

  std::cout << "\n\nStarting okvis processing... \n\n";

  okvis::Time startTime = okvis::Time::now();

  int progress = 0;
  bool datasetreaderFinished = false;
  while(true){

    if(!datasetreaderFinished){
      okvis_estimator->processFrame();
      okvis_estimator->display();
      writer->drawTopView();
      cv::waitKey(2);
    }

    // check if done
    if(!datasetReader->isStreaming()) {
      datasetreaderFinished = true;
      std::cout << "\r DatasetReader Finished!" << std::endl;

      if(datasetreaderFinished && se_interface->finishedIntegrating()){
        if(parameters.estimator.do_final_ba) {
          LOG(INFO) << "final full BA...";
          okvis_estimator->doFinalBa();
          cv::waitKey(1);
        }
        LOG(INFO) <<"total processing time OKVIS only " << (okvis::Time::now() - startTime) << " s" << std::endl;
        okvis_estimator->writeFinalTrajectoryCsv();
        okvis_estimator->writeGlobalTrajectoryCsv(path+"/okvis2-" + mode + "-global-final_trajectory.csv");
        okvis_estimator->saveMap(); // This saves landmarks map
        LOG(INFO) <<"total processing time " << (okvis::Time::now() - startTime) << " s" << std::endl;
        cv::waitKey(1);
        break;
      }
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

}

int main(int argc, char **argv)
{

  SubmapInterfacer submapper(argv);

  submapper.start();

  return EXIT_SUCCESS;
}
