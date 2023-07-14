//
// Created by boche on 10/7/22.
//
#include <okvis/SubmappingInterface.hpp>
#include <chrono>
#include <queue>
#include "se/map/io/point_cloud_io.hpp"

namespace okvis {

  void saveDepthImages(DepthFrame& dImage, se::PinholeCamera& sensor, std::string filename){

    Eigen::Vector2i size(dImage.width(), dImage.height());
    se::save_depth_png(dImage.data(), size, filename);

  }

  bool depthImgResDownsampling(DepthFrame& originalImage, DepthFrame& downsampledImage, int downsamplingFactor){
    if(originalImage.width() % downsamplingFactor == 0 && originalImage.height() % downsamplingFactor == 0){
      se::preprocessor::downsample_depth(originalImage, downsampledImage);
      return true;
    }
    else{
      LOG(WARNING) << "Invalid downsampling factor, providing original image";
      downsampledImage = originalImage;
      return false;
    }
  }

  void SubmappingInterface::generatePointCloudFromLidar(const std::vector<std::pair<Eigen::Matrix4f, Eigen::Vector3f>,
          Eigen::aligned_allocator<std::pair<Eigen::Matrix4f, Eigen::Vector3f>>>& lidarMeas, int counter){
        
        se::Image<Eigen::Vector3f> pc(lidarMeas.size(), 1);
        Eigen::Matrix4f T_WS = Eigen::Matrix4f::Identity();

        for(size_t i = 0; i < lidarMeas.size(); i++){
          pc[i] = (lidarMeas[i].first * lidarMeas[i].second.homogeneous()).head<3>();
        }

        std::string filename = meshesPath_ + "/../lidarPc/" + std::to_string(counter) + ".vtk";
        
        int test = save_point_cloud_vtk(pc, filename, T_WS);

        if(test == 0) LOG(INFO) << "Correctly saved pointcloud in file " + filename;
        else LOG(INFO) << "Incorrectly saved pointcloud";

    }

  void SubmappingInterface::setGoal(const Eigen::Matrix4d& T_BBg) {
    LOG(INFO) << "Received goal:\n" << T_BBg;
    std::lock_guard _(goal_mtx_);
    goal_T_BBg_ = T_BBg;
    has_goal_ = true;
  }

  /// \brief Random "downsampling" of a pointcloud (vector of points)
  /// \param originalPointCloud       Original Point Cloud
  /// \param downsampledPointCloud    Downsampled Point Cloud
  /// \param num_of_points            Number of points in the resulting point cloud
  void downsamplePointCloud(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& originalPointCloud,
                            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& downsampledPointCloud,
                            size_t num_of_points)
  {
    std::sample(originalPointCloud.begin(), originalPointCloud.end(), std::back_inserter(downsampledPointCloud), num_of_points,
                std::mt19937{std::random_device{}()});
  }

  bool SubmappingInterface::addLidarMeasurement(const okvis::Time &stamp,
                                                  const Eigen::Vector3d &ray) { // ToDo: adjust logic for lidar (consider large number of measurements)
    // Create an OKVIS LiDAR measurement.
    LidarMeasurement lidarMeasurement;
    lidarMeasurement.timeStamp = stamp;
    lidarMeasurement.measurement.rayMeasurement = ray;

    // Push data to the Queue.
    const size_t lidarQueueSize =100000000; ///< Arbitrary number. ToDo -> fine-tune
    sensorMeasurementDownsamplingCounter_++;
        
    if(sensorMeasurementDownsamplingCounter_ % sensorMeasurementDownsampling_ == 0){
      sensorMeasurementDownsamplingCounter_ = 0;
      if (blocking_) { // ToDo: Remove blocking behavior?
        const bool result =
                lidarMeasurements_.PushBlockingIfFull(lidarMeasurement, lidarQueueSize);
        return result;
      } else {
        // Push measurement and pop the oldest entry.
        const bool result = lidarMeasurements_.PushNonBlockingDroppingIfFull(
                lidarMeasurement, lidarQueueSize);
        if (result)
          LOG(WARNING) << "Oldest LiDAR measurement dropped";
          return true;
        }
    }

    return true;

  }

    bool SubmappingInterface::addDepthMeasurement(const okvis::Time &stamp,
                                                  const cv::Mat &depthImage){

        CameraMeasurement depthMeasurement;
        depthMeasurement.timeStamp = stamp;
        depthMeasurement.measurement.depthImage = cameraSensor_->second.scale * depthImage;

        const size_t depthQueueSize = 4; //We want to buffer 4 depth images to give OKVIS to have time to compute the states
        sensorMeasurementDownsamplingCounter_++;

        if(sensorMeasurementDownsamplingCounter_ % sensorMeasurementDownsampling_ == 0){
          sensorMeasurementDownsamplingCounter_ = 0;
          if(blocking_){
            bool result = depthMeasurements_.PushBlockingIfFull(depthMeasurement, depthQueueSize);
            return result;
          }else{
            bool result = depthMeasurements_.PushNonBlockingDroppingIfFull(depthMeasurement, depthQueueSize);

            if(result) {

              LOG(WARNING) << "Oldest depth image dropped";
            }
              
            return true;
          }
        }
        
        return true;

    }

    void SubmappingInterface::integrationLoop(){
      
      LOG(INFO) << "in the integration loop";
      while(!isFinished()){
        if(lidarMeasurements_.Size() > 0 || depthMeasurements_.Size() > 0){
          checkForAvailableData();
        }
      }
    }


    bool SubmappingInterface::finishedIntegrating(){
      uint64_t numMessages;
      numMessages = supereightFrames_.Size();
      LOG(INFO) << "Number of lidar points " + std::to_string(lidarMeasurements_.Size()) + " number of supereight frames " 
                   + std::to_string(supereightFrames_.Size()) + " number of states " + std::to_string(stateUpdates_.Size()) << std::endl;
      
      if(numMessages == 0 && submapPoseLookup_.size() > 0){

        if(frame_){
          frame_->endSubmap = true;
          if(blocking_){
            supereightFrames_.PushBlockingIfFull(*frame_,1);
          }
          else{
            supereightFrames_.PushNonBlockingDroppingIfFull(*frame_, 2);
          }
          frame_ = nullptr;
          return finishedIntegrating();
        }
        
        setFinished();
        return true;

      }

      return false;
    }

    void SubmappingInterface::frameUpdater(okvis::OkvisUpdate& propagatedStateUpdate) {

      constexpr const size_t supereightQueueSize = 5000;

      if (propagatedStateUpdate.isKeyframe || !latestKeyframe.started) { // first update should always be keyframe, but lets be safe
        latestKeyframe.started = true;

        if(propagatedStateUpdate.latestState.id.value() != latestKeyframe.Id) {
          if(kfCounter_.size() != kfThreshold_){
            kfCounter_.insert(propagatedStateUpdate.latestState.id.value());
          }else{
            kfCounter_.clear();
            kfCounter_.insert(propagatedStateUpdate.latestState.id.value());
            latestKeyframe.Id = propagatedStateUpdate.latestState.id.value();
          }
        }

      }

      KeyFrameDataMap keyFrameDataMap;

      for (size_t i = 0; i < propagatedStateUpdate.keyframeStates.size(); i++) {
        const auto &state = propagatedStateUpdate.keyframeStates[i];
        keyFrameDataMap[state.id.value()] = KeyframeData(state.id.value(), state.T_WS);
      }

      // Is current state a loop closure state?
      bool loop_closure = propagatedStateUpdate.loop_closure;

      if(!frame_) {
        frame_ = new SupereightFrames(latestKeyframe.Id, keyFrameDataMap, loop_closure);
      } else if(frame_->keyFrameId != latestKeyframe.Id){

        frame_->endSubmap=true;

        if(blocking_){
          supereightFrames_.PushBlockingIfFull(*frame_, 1);
        }
        else{
          if(supereightFrames_.PushNonBlockingDroppingIfFull(*frame_, supereightQueueSize)){
            LOG(WARNING) << "Oldest Supereight frame dropped";
          }else{
            LOG(INFO) << "New place Set a supereight frame with ID " + std::to_string(frame_->keyFrameId);
          }
        }
        delete frame_;
        frame_ = new SupereightFrames(latestKeyframe.Id, keyFrameDataMap, loop_closure);
        if(frame_->loop_closure == false && loop_closure){
          frame_->loop_closure = true;
        }
        updateSEKeyFrameData(frame_->kfData_, keyFrameDataMap); 
      } else {
        if(frame_->loop_closure == false && loop_closure){
          frame_->loop_closure = true;
        } 
        updateSEKeyFrameData(frame_->kfData_, keyFrameDataMap);
      }

    }

    template<typename DepthSource>
    void SubmappingInterface::depthFrameUpdater(Measurement<DepthSource>& depthMeasurement, Eigen::Matrix4f& T_WD){

      static_assert(std::is_same_v<DepthSource, LidarSensorReadings> || std::is_same_v<DepthSource, CameraData>, "not implemented for this sensor type");

      if(!frame_){
        throw std::runtime_error("In depthFrameUpdater we have found a point where there is no frame initialized where there should have be");
      }else{
          if constexpr (std::is_same_v<DepthSource, LidarSensorReadings>){ //constexpr to ensure it is done at compile time
            frame_->vecRayMeasurements.push_back(std::make_pair(T_WD, depthMeasurement.measurement.rayMeasurement.template cast<float>()));
          } else if constexpr (std::is_same_v<DepthSource, CameraData>){
            frame_->vecDepthFrames.push_back(std::make_pair(T_WD, depthMeasurement.measurement.depthImage));
          }
      }

    }

    bool SubmappingInterface::predict(const okvis::Time &finalTimestamp,
                                      Transformation &T_WC,
                                      bool isCamera) {

      // Get the okvis update closest to the finalTimestamp (the stamp of the depth frame)
      // We use Okvis::trajectory to propagate state. Also need to use the updates queue bc we also need
      // Updated kfs and all that other stuff that we need to build supereight frames

      const Transformation& T_SD = isCamera? cameraSensor_->first : lidarSensor_->first;

      okvis::State measurementState;
      const kinematics::Transformation* T_WS; // = propagatedStateUpdate.latestState.T_WS;

      TimerSwitchable getState("8.1.3 getting intermediate state");
      if(useGtPoses_ && gtPoses_.find(finalTimestamp.toNSec()) != gtPoses_.end()){
          T_WS = &gtPoses_[finalTimestamp.toNSec()];
      }else{
        if(propagatedStates.getState(finalTimestamp, measurementState)){
          T_WS = &measurementState.T_WS;
        }else{
          return false;
        }
      }

      getState.stop();

      T_WC = (*T_WS) * T_SD;
      return true;

    }


    void SubmappingInterface::saveSubmap(uint64_t kfId){
      LOG(INFO) << "In saving submap independent thread";
      const std::string meshFilename = meshesPath_ + "/mesh_kf" + std::to_string(kfId) + ".ply";
      LOG(INFO) << "Testing";
      std::cout << "Saving to path " + meshFilename << std::endl;
      LOG(INFO) << "Testing2";

      auto start = std::chrono::high_resolution_clock::now();
      submapLookup_[kfId]->saveMesh(meshFilename, (submapPoseLookup_[kfId].T()).cast<float>());      
      auto end_first = std::chrono::high_resolution_clock::now();
      
      std::cout << "It took " << std::to_string(std::chrono::duration_cast<std::chrono::seconds>(end_first - start).count()) << " seconds to process the mesh" << std::endl;

    }

    void SubmappingInterface::setT_BS(const okvis::kinematics::Transformation& T_BS) {
      T_BS_ = T_BS;
      T_SB_ = T_BS_.inverse();
    }

    void SubmappingInterface::processSupereightFrames() {

      // Get Supereight Frames --> supposed to be integrated into submaps
      SupereightFrames supereightFrame;
      SubmapAlignmentTerm submapAlignBlock;
      while(!isFinished()){
        while (supereightFrames_.PopBlocking(&supereightFrame)){

          std::set<uint64_t> updated_maps; //Stores kf indexes of modified/updated submaps
          updated_maps.insert(supereightFrame.keyFrameId);

          for (auto &keyframeData: supereightFrame.kfData_) {

            //We only update the poses of the keyframes which have a submap associated to it
            const Transformation T_WS = keyframeData.second.T_WS;

            // Check If Id exists.

            if (submapPoseLookup_.count(keyframeData.first)) {
              updated_maps.insert(keyframeData.first);
              submapPoseLookup_[keyframeData.first] = T_WS;
            } else if(keyframeData.first == supereightFrame.keyFrameId){
              // Insert
              submapPoseLookup_.insert(std::make_pair(keyframeData.first, T_WS));
            }
          }

          // create new map
          static unsigned int submap_counter = 0;
          submap_counter++;

          // Create a new submap and reset frame counter
          if(prevKeyframeId_ != supereightFrame.keyFrameId){

            std::cout << "New submap no. " << submap_counter << " (kf Id: " << supereightFrame.keyFrameId << ")" << "\n";

            submapLookup_.insert(std::make_pair(supereightFrame.keyFrameId, std::shared_ptr<se::OccupancyMap<se::Res::Multi>>
                                                                            (new se::OccupancyMap<se::Res::Multi>(mapConfig_, dataConfig_))));

            // do a preliminary hashing (allocate 10x10x10 box in hash table)
            // we do this so that we can plan even while integrating current map
            /*
            std::thread prelim_hashing_thread(&SubmappingInterface::doPrelimSpatialHashing, this,
                                              supereightFrame.keyFrameId,
                                              submapPoseLookup_[supereightFrame.keyFrameId].r());
            prelim_hashing_thread.detach();
            */
            SubmappingInterface::SpatialHashing(supereightFrame.keyFrameId, submapPoseLookup_[supereightFrame.keyFrameId], submapLookup_[supereightFrame.keyFrameId]);

            // now we integrate in this keyframe, until we find a new one that is distant enough
            prevKeyframeId_ = supereightFrame.keyFrameId;
          }

          auto& activeMap = *submapLookup_[supereightFrame.keyFrameId];

          okvis::Time tLookUpMap = okvis::Time::now();

          se::MapIntegrator integrator(activeMap);
          okvis::Time tInitIntegrator = okvis::Time::now();
          
          TimerSwitchable batchIntegration("8.4 Batch integrating");

          if(lidarSensor_){
            //LOG(INFO) << "The amount of LiDAR points is " + std::to_string(supereightFrame.vecRayMeasurements.size());

            okvis::kinematics::Transformation T_WK = submapPoseLookup_[prevKeyframeId_];
            LOG(INFO) << "Before doing the transform";
            std::transform(std::begin(supereightFrame.vecRayMeasurements), std::end(supereightFrame.vecRayMeasurements),
                            std::begin(supereightFrame.vecRayMeasurements),
                            [&T_WK](auto& pair){return std::pair<Eigen::Matrix4f, Eigen::Vector3f> (T_WK.inverse().T().cast<float>() * pair.first,pair.second);});
            LOG(INFO) << "About to integrate ray batch";
            integrator.integrateRayBatch(lidarSensor_->second, supereightFrame.vecRayMeasurements, integration_counter_);
            // add measurements to alignmentblock
            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> onlyRays;
            std::transform(std::begin(supereightFrame.vecRayMeasurements), std::end(supereightFrame.vecRayMeasurements),
                            std::back_inserter(onlyRays), [](auto const& pair){ return pair.second; });

            
            integration_counter_++;
          }
          
          
          batchIntegration.stop();


          TimerSwitchable diIntegration("8.5 DI integration");

          if(cameraSensor_){
            //LOG(INFO) << "The amount of Depth frames is " + std::to_string(supereightFrame.vecDepthFrames.size());
            okvis::kinematics::Transformation T_WK = submapPoseLookup_[prevKeyframeId_];

            for(size_t i = 0; i < supereightFrame.vecDepthFrames.size(); i++){
              integrator.integrateDepth(cameraSensor_->second, depthMat2Image(supereightFrame.vecDepthFrames[i].second),
              T_WK.inverse().T().cast<float>() * supereightFrame.vecDepthFrames[i].first, integration_counter_);
              integration_counter_++;
            }
            
          }
          
          diIntegration.stop();

          // Execute spatial hashing
          //for (auto it = updated_maps.begin(); it != updated_maps.end(); it++)
          for (auto it : updated_maps)
          {
            TimerSwitchable Spatialhashing("8.6 Spatial hashing");
            SubmappingInterface::SpatialHashing(it, submapPoseLookup_[it], submapLookup_[it]);
            Spatialhashing.stop();
          }

          //Do it with depth images

          // Get the latest state estimate if one exists.
          okvis::State current_state;
          const bool current_state_valid = !propagatedStates.stateIds().empty()
                                             && propagatedStates.getState(*propagatedStates.stateIds().rbegin(), current_state);

          if (fieldCallback_ && current_state_valid) {
            fieldCallback_(current_state, submapPoseLookup_, submapLookup_);
          }

          
          // Check if there's a new goal pose to plan to.
          bool has_goal = false;
          Eigen::Matrix4d T_BBg;
          {
            std::lock_guard _(goal_mtx_);
            if (has_goal_) {
              T_BBg = goal_T_BBg_;
              has_goal = true;
              has_goal_ = false;
            }
          }
          
          if (has_goal && current_state_valid) {
            LOG(INFO) << "Has goal: " << (has_goal ? "yes" : "no");
            // NOTE: Assuming frame S is in the center of the MAV and x-forward, z-up.
            const Eigen::Matrix4d T_WB = (current_state.T_WS * T_SB_).T();
            const Eigen::Matrix4d T_WBg = T_WB * T_BBg;
            LOG(INFO) << "Relative goal T_BBg:\n" << T_BBg
              << "\nconverted to absolute goal T_WBg:\n" << T_WBg;
            const Eigen::Quaterniond q_WB(T_WB.topLeftCorner<3,3>());
            const Eigen::Quaterniond q_WBg(T_WBg.topLeftCorner<3,3>());
            const Eigen::Vector3f start = T_WB.topRightCorner<3,1>().cast<float>();
            const Eigen::Vector3f end = T_WBg.topRightCorner<3,1>().cast<float>();
            // std::vector<se::planning::Submap<se::OccupancyMap<se::Res::Multi>>,
            //             Eigen::aligned_allocator<se::planning::Submap<se::OccupancyMap<se::Res::Multi>>>> maps;
            se::planning::SubmapMap<uint64_t, se::OccupancyMap<se::Res::Multi>> mapped_maps;
            for(auto& it : submapLookup_){
              mapped_maps.emplace(std::make_pair(it.first,
                                  se::planning::Submap<se::OccupancyMap<se::Res::Multi>>(it.second.get(), (submapPoseLookup_[it.first].T()).cast<float>())));
            }
            se::planning::PlannerConfig planner_config;
            planner_config.solving_time = 0.3; // 0.1;
            planner_config.robot_radius = 0.5;
            se::planning::PathPlanner<uint64_t, se::OccupancyMap<se::Res::Multi>> planner(hash_cell_size, hashTable_, mapped_maps, planner_config);
            LOG(INFO) << "Planner started";
            const auto res = planner.plan(start, end);
            LOG(INFO) << "Planner status: " << res;
            if (res && pathPlannerCallback_) {
              const auto& positions = planner.path();
              std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> path (positions.size(), Eigen::Matrix4d::Identity());
              for (size_t i = 0; i < path.size(); i++) {
                path[i].topRightCorner<3,1>() = positions[i].cast<double>();
                // The orientation of intermediate path poses is interpolated
                // between the current and goal orienations.
                const double a = static_cast<double>(i) / (path.size() - 1);
                path[i].topLeftCorner<3,3>() = q_WB.slerp(a, q_WBg).toRotationMatrix();
              }
              pathPlannerCallback_(path);
            }
          }
          

          if(supereightFrame.endSubmap){

            std::cout << "Completed integrating submap " << prevKeyframeId_ << " which is submap number " << submapLookup_.size() << "\n";
            LOG(INFO) << okvis::timing::Timing::print();
            
            // do spatial hashing when the whole submap is finished
            SubmappingInterface::SpatialHashing(supereightFrame.keyFrameId, submapPoseLookup_[supereightFrame.keyFrameId], submapLookup_[supereightFrame.keyFrameId]);
            /*
            // Set the goal and plan the path
            if (current_state_valid) {
              // NOTE: Assuming frame S is in the center of the MAV and x-forward, z-up.
              //const Eigen::Matrix4d T_WB = (current_state.T_WS * T_SB_).T(); // Set the current robot position as the start position
              const Eigen::Matrix4d T_WB = submapPoseLookup_[supereightFrame.keyFrameId].T().cast<double>(); // Set the current submap pose as the start position
              // Set the goal as 1m forward in body frame
              Eigen::Matrix4d T_BBg;
              T_BBg << 1.d, 0.d, 0.d, 0.d,
                       0.d, 1.d, 0.d, 0.d,
                       0.d, 0.d, 1.d, 1.d,
                       0.d, 0.d, 0.d, 0.d;
              const Eigen::Matrix4d T_WBg = T_WB + T_WB * T_BBg;
              LOG(INFO) << "Relative goal T_BBg:\n" << T_BBg
                << "\nconverted to absolute goal T_WBg:\n" << T_WBg;
              const Eigen::Quaterniond q_WB(T_WB.topLeftCorner<3,3>());
              const Eigen::Quaterniond q_WBg(T_WBg.topLeftCorner<3,3>());
              const Eigen::Vector3f start = T_WB.topRightCorner<3,1>().cast<float>();
              const Eigen::Vector3f end = T_WBg.topRightCorner<3,1>().cast<float>();
              // std::vector<se::planning::Submap<se::OccupancyMap<se::Res::Multi>>,
              //             Eigen::aligned_allocator<se::planning::Submap<se::OccupancyMap<se::Res::Multi>>>> maps;
              se::planning::SubmapMap<uint64_t, se::OccupancyMap<se::Res::Multi>> mapped_maps;
              for(auto& it : submapLookup_){
                mapped_maps.emplace(std::make_pair(it.first,
                                    se::planning::Submap<se::OccupancyMap<se::Res::Multi>>(it.second.get(), (submapPoseLookup_[it.first].T()).cast<float>())));
              }
              se::planning::PlannerConfig planner_config;
              planner_config.solving_time = 0.8; // 0.1;
              planner_config.robot_radius = 0.5;
              se::planning::PathPlanner<uint64_t, se::OccupancyMap<se::Res::Multi>> planner(hash_cell_size, hashTable_, mapped_maps, planner_config);
              LOG(INFO) << "Planner started";
              const auto res = planner.plan(start, end);
              LOG(INFO) << "Planner status: " << res;
              if (res && pathPlannerCallback_) {
                const auto& positions = planner.path();
                std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> path (positions.size(), Eigen::Matrix4d::Identity());
                for (size_t i = 0; i < path.size(); i++) {
                  path[i].topRightCorner<3,1>() = positions[i].cast<double>();
                  // The orientation of intermediate path poses is interpolated
                  // between the current and goal orienations.
                  const double a = static_cast<double>(i) / (path.size() - 1);
                  path[i].topLeftCorner<3,3>() = q_WB.slerp(a, q_WBg).toRotationMatrix();
                }
                pathPlannerCallback_(path);
              }
            }
            */
            
            // save mesh and publish submap
            if (submapMeshesCallback_ || submapCallback_) {
              AlignedUnorderedMap<uint64_t, Transformation> submapPoses;
              std::unordered_map<uint64_t, std::shared_ptr<se::OccupancyMap<se::Res::Multi>>> submaps;

              for(auto it: updated_maps){
                submapPoses[it] = submapPoseLookup_[it];
                submaps[it] = submapLookup_[it];
              }
              
              TimerSwitchable submap_viz("8.6 Submap visualization");

              publishSubmaps(submapPoses, submaps);
              submap_viz.stop();
            }
          }else{
          }

          //LOG(INFO) << okvis::timing::Timing::print();

        }
      }

      return;

    }

    void SubmappingInterface::generatePointCloud(const Eigen::Matrix4f& T_WS, const DepthFrame& dImage, const se::PinholeCamera& sensor, int counter){

        std::cout << "T_WS matrix is: " << std::endl;
        std::cout << T_WS << std::endl;
        std::cout << "Camera parameters are: fx= " << sensor.model.focalLengthU() << " fy= " << sensor.model.focalLengthV() << " cx= " << sensor.model.imageCenterU() << " cy= " << sensor.model.imageCenterV() << std::endl;  
        se::Image<Eigen::Vector3f> pc(sensor.model.imageWidth(), sensor.model.imageHeight());
        std::string filename = meshesPath_ + "/../pc" + std::to_string(counter) + ".vtk";
        std::string diFilename = meshesPath_ + "/../depth_images/dI" + std::to_string(counter) + ".png";
        Eigen::Vector2i size(dImage.width(), dImage.height());
        se::save_depth_png(dImage.data(), size, diFilename);
        se::preprocessor::depth_to_point_cloud(pc, dImage, sensor);
        int test = save_point_cloud_vtk(pc, filename, T_WS);

        if(test == 0) LOG(INFO) << "Correctly saved pointcloud in file " + filename;
        else LOG(INFO) << "Incorrectly saved pointcloud";

    }

    DepthFrame SubmappingInterface::depthMat2Image(const cv::Mat &inputDepth) {

      // Initialise and copy
      if(inputDepth.type() != CV_32FC1) {
        throw std::runtime_error("Only implemented for CV_32FC1 cv::Mat");
      }
      DepthFrame output(inputDepth.cols, inputDepth.rows);

      // cv::MAT and DepthFrame keep data stored in row major format.
      if(!inputDepth.isContinuous()) {
        //TODO write down row by row, first iterate rows the columns and add them to a vector
        throw std::runtime_error("Only implemented for continuous cv::Mat");
      }

      memcpy(output.data(), inputDepth.data,
            inputDepth.cols * inputDepth.rows * sizeof(float));

      return output;

    }

    bool SubmappingInterface::checkForAvailableData() {

      CameraMeasurement oldestDepthImage;
      LidarMeasurement oldestLidarMeasurement;
      bool integrateLidar = true;
      bool integrateDepthImage = true;

      // Check if any measurements in the queue to be integrated
      if (!lidarMeasurements_.getCopyOfFront(&oldestLidarMeasurement) && !depthMeasurements_.getCopyOfFront(&oldestDepthImage)){
        //std::cout << "no lidarMeasurements and depthMeasurements in the queues" << std::endl;
        return false;
      }

      // Check if State Updates available (if so, get neweset State).
      OkvisUpdate newestState;
      if(!stateUpdates_.getCopyOfBack(&newestState)){
        //std::cout << "not having any state updates" << std::endl;
        return false;
      }

      // Check if oldest Measurement to be integrated is older than the newest State.
      if(!(oldestLidarMeasurement.timeStamp <= newestState.timestamp) || !lidarSensor_){
        //std::cout << "not having sufficiently recent state updates" << std::endl;
        integrateLidar = false;
      }

      if(!(oldestDepthImage.timeStamp <= newestState.timestamp) || !cameraSensor_){
        integrateDepthImage = false;
      }

      if(!(integrateDepthImage || integrateLidar)){
        //LOG(INFO) << "No integration going on due to lack of Lidar data"; 
        OkvisUpdate popState;
        bool continuePopping = true;
        while(continuePopping){
          

          if(blocking_) {
            continuePopping = stateUpdates_.PopBlocking(&popState);
          } else {
            continuePopping = stateUpdates_.PopNonBlocking(&popState);
          }

          propagatedStates.update(popState.trackingState, popState.updatedStates, popState.affectedStates);
          frameUpdater(popState);

          if(continuePopping) {
            continuePopping = popState.timestamp < newestState.timestamp? true: false;
          }

        }
        
        return false;
      } 

      // Get oldest state update to iterate
      OkvisUpdate currentStateUpdate;
      if(blocking_) {
        stateUpdates_.PopBlocking(&currentStateUpdate);
      }
      else {
        stateUpdates_.PopNonBlocking(&currentStateUpdate);
      }
      // Update internal representation of the trajectory
      propagatedStates.update(currentStateUpdate.trackingState, currentStateUpdate.updatedStates, currentStateUpdate.affectedStates);

      if(!frame_) {
        frameUpdater(currentStateUpdate);
      }
      

      // Setting some variables required for looping over state updates and measurements
      bool keepLooping = true;
      const size_t supereightQueueSize = 5000; /// Large Queue needed for LiDAR measurements if we do not want to drop anything.
      Transformation T_WD;
      KeyFrameDataMap keyFrameDataMap;

      // Iterate available measurements
      LidarMeasurement lidarMeasurement;
      lidarMeasurements_.getCopyOfFront(&lidarMeasurement);
      CameraMeasurement depthMeasurement;
      depthMeasurements_.getCopyOfFront(&depthMeasurement);

      while(keepLooping){

        if(integrateLidar && lidarMeasurement.timeStamp <= currentStateUpdate.timestamp){

          if(blocking_) {
            lidarMeasurements_.PopBlocking(&lidarMeasurement);
          }
          else {
            lidarMeasurements_.PopNonBlocking(&lidarMeasurement);
          }

          TimerSwitchable predTimer("8.1 predicting intermediate positions LiDAR");

          if(!predict(lidarMeasurement.timeStamp, T_WD, false)){
            predTimer.stop();
            continue;
          }
          predTimer.stop();

          TimerSwitchable predTimerSE("8.3 Generating or updating supereight frames");

          Eigen::Matrix4f T = T_WD.T().cast<float>();
          depthFrameUpdater(lidarMeasurement, T);

          predTimerSE.stop();
        }
        if(integrateDepthImage && depthMeasurement.timeStamp <= currentStateUpdate.timestamp){

          if(blocking_) {
            depthMeasurements_.PopBlocking(&depthMeasurement);
          }
          else {
            depthMeasurements_.PopNonBlocking(&depthMeasurement);
          }

          TimerSwitchable predTimer2("8.2 predicting intermediate positions depth images");
          if(!predict(depthMeasurement.timeStamp, T_WD)){
            LOG(WARNING) << "Depth image dropped with timestamp " << depthMeasurement.timeStamp;
            predTimer2.stop();
            continue;
          }
          predTimer2.stop();

          TimerSwitchable predTimerSE("8.3 Generating or updating supereight frames");

          if(cameraSensor_ && depthImageResDownsamplingRatio_ > 1){
            cv::Mat subsampled;
            cv::resize(depthMeasurement.measurement.depthImage, subsampled, cv::Size(), 1.0 / depthImageResDownsamplingRatio_, 1.0 / depthImageResDownsamplingRatio_, cv::INTER_NEAREST);
            depthMeasurement.measurement.depthImage = subsampled;
          }

          Eigen::Matrix4f T = T_WD.T().cast<float>();
          depthFrameUpdater(depthMeasurement, T);

          predTimerSE.stop();

        }
        if( (!depthMeasurements_.getCopyOfFront(&depthMeasurement) || depthMeasurement.timeStamp >= currentStateUpdate.timestamp || !integrateDepthImage)
          && (!lidarMeasurements_.getCopyOfFront(&lidarMeasurement) || lidarMeasurement.timeStamp >= currentStateUpdate.timestamp || !integrateLidar)){
          keepLooping = false;

          if(frame_ && latestKeyframe.Id != frame_->keyFrameId){
            std::cout << "From the states it is " + std::to_string(latestKeyframe.Id) + " from the frame it is " + std::to_string(frame_->keyFrameId) << std::endl;
            frame_->endSubmap=true;
            if(blocking_){
              supereightFrames_.PushBlockingIfFull(*frame_, 1);
            }
            else{
              if(supereightFrames_.PushNonBlockingDroppingIfFull(*frame_, supereightQueueSize)){
                LOG(WARNING) << "Oldest Supereight frame dropped";
              }else{
                //LOG(INFO) << "Set a supereight frame with ID " + std::to_string(frame_->keyFrameId);
              }
            }
            frame_ = nullptr;
          }else if(frame_){
            frame_->stateCounter_++;
          }

          frameUpdater(currentStateUpdate);

          if(frame_ && frame_->stateCounter_ >= stateThreshold_){
            if(blocking_){
              supereightFrames_.PushBlockingIfFull(*frame_, 1);
            } else if(supereightFrames_.PushNonBlockingDroppingIfFull(*frame_, supereightQueueSize)){
              LOG(WARNING) << "Oldest Supereight frame dropped";
            }else{
              //LOG(INFO) << "Sent a partial supereight submap with ID " + std::to_string(frame_->keyFrameId);
            }

            frame_->stateCounter_ = 0;
            frame_->vecRayMeasurements.clear();
            frame_->vecDepthFrames.clear();
          }
        }
      }
      return true;
    }

    bool SubmappingInterface::start() {

      std::cout << "\n\nStarting supereight processing... \n\n";

      std::ofstream file("/home/stud/zxuh/datasets/results/collision_checking/checking_time_num_maps.csv", std::ofstream::app);
      if (!file.is_open()) {
      std::cerr << "Unable to save checking time " << "\n";
      return false;
      }

      dataIntegration_ = std::thread(&SubmappingInterface::integrationLoop, this);
      submapIntegration_ = std::thread(&SubmappingInterface::processSupereightFrames, this);

      return true;
    }
    
    bool SubmappingInterface::stateUpdateCallback(
            const State &latestState, const TrackingState &latestTrackingState,
            std::shared_ptr<const okvis::AlignedMap<StateId, State>> updatedStates) {

      if(latestTrackingState.isKeyframe){
        keyframes_.insert(latestTrackingState.id);
      }

      StateVector keyframeStatesOnly;
      std::set<StateId> affectedStateIds;
      auto it = updatedStates->begin();
      while(it != updatedStates->end()){
        affectedStateIds.insert(it->first);
        if(keyframes_.find(it->first) != keyframes_.end()){
          keyframeStatesOnly.push_back(it->second);
        }
        ++it;
      }


      OkvisUpdate latestStateData(latestState,
                                  keyframeStatesOnly,
                                  latestState.timestamp,
                                  latestTrackingState.isKeyframe,
                                  latestTrackingState.currentKeyframeId.value(),
                                  latestTrackingState.recognisedPlace,
                                  latestTrackingState,
                                  affectedStateIds,
                                  updatedStates);
      const size_t stateUpdateQueue = 100000;

      if(latestStateData.keyframeStates.empty())
        return false;

      if (blocking_) {
        bool success = stateUpdates_.PushBlockingIfFull(latestStateData, 1);
        return success;
      } else {
        if (stateUpdates_.PushNonBlockingDroppingIfFull(latestStateData,
                                                        stateUpdateQueue)) {
          LOG(WARNING) << "Oldest state  measurement dropped";
          return true;
        }
      }

      return false;
    }

    void SubmappingInterface::publishSubmaps(AlignedUnorderedMap<uint64_t, Transformation> submapPoses,
                                             std::unordered_map<uint64_t, std::shared_ptr<se::OccupancyMap<se::Res::Multi>>> submaps){

      std::cout << "publishing submaps" << std::endl;

      if (submapCallback_) {
        submapCallback_(submapPoses, submaps);
      }

      if (submapMeshesCallback_) {
        std::thread publish_meshes(submapMeshesCallback_, submapPoseLookup_);
        publish_meshes.detach();
      }
    }
/*
    void SubmappingInterface::redoSpatialHashing(const uint64_t id, const Transformation& Tf,
                                                 const std::shared_ptr<se::OccupancyMap<se::Res::Multi>> map) {

      std::unique_lock<std::mutex> lk(hashTableMutex_);

      Eigen::Matrix4f T_KM = map->getTWM();
      Eigen::Matrix4d T_WK = Tf.T();
      Eigen::Matrix4f T_WM = T_WK.cast<float>() * T_KM;

      // sanity checks
      assert(submapDimensionLookup_.count(id) && hashTableInverse_.count(id));

      // get map bounds
      Eigen::Matrix<float, 6, 1> bounds = submapDimensionLookup_[id];

      Eigen::Vector3f min_box_metres = {bounds(0), bounds(1), bounds(2)};
      Eigen::Vector3f max_box_metres = {bounds(3), bounds(4), bounds(5)};

      // remove boxes for "id" submap
      for (auto &pos: hashTableInverse_[id]) {
        hashTable_[pos].erase(id); // remove id from box
        hashTableInverse_[id].erase(pos); // remove box from id
      }

      const float side = 1.0; // hardcoded hash map box side of 1m
      const float step = 0.5 * side * sqrt(2); // this ensures full cover of submap space


      for (float x = min_box_metres(0); x <= max_box_metres(0); x += step) {
        for (float y = min_box_metres(1); y <= max_box_metres(1); y += step) {
          for (float z = min_box_metres(2); z <= max_box_metres(2); z += step) {

            // get offset value (this pos is in map frame)
            Eigen::Vector4f pos_map(x, y, z, 1);

            // transform into world frame
            Eigen::Vector4f pos_world;
            pos_world = T_WM * pos_map;

            // floor transformed value
            Eigen::Vector3i pos_floor;
            for (int i = 0; i < 3; i++) {
              // if side is e.g. 2, a value of 4.5,4.5,4.5 is mapped to box 2,2,2
              pos_floor(i) = (int) (floor(pos_world(i) / side));
            }

            // add to index
            hashTable_[pos_floor].insert(id);

            // add pos to inverse index
            hashTableInverse_[id].insert(pos_floor);
          }
        }
      }

      lk.unlock();

    }
*/
/*
// pass by value needed
    void SubmappingInterface::doPrelimSpatialHashing(const uint64_t id, const Eigen::Vector3d& pos_kf) {

      const int side = 1; // step (in metre)s of the spatial grid
      const int box_side = 10; // dim of the box we'll allocate

      // box dims, in metres
      Eigen::Vector3i min_box_metres = Eigen::Vector3i::Constant(-box_side);
      Eigen::Vector3i max_box_metres = Eigen::Vector3i::Constant(box_side);
      Eigen::Matrix<int, 6, 1> dims;
      dims << min_box_metres, max_box_metres;

      // box dims, in box units (not metres)
      Eigen::Vector3i min_box;
      Eigen::Vector3i max_box;

      // create 10x10x10m box around kf pos. round values to box units
      for (int i = 0; i < 3; i++) {
        min_box(i) = floor((pos_kf(i) - box_side) / side);
        max_box(i) = floor((pos_kf(i) + box_side) / side);
      }

      std::unique_lock<std::mutex> lk(hashTableMutex_);

      // add dimensions in lookup
      // should be relative to map frame but who cares... this is just a big box
      // this needs to be in metres instead
      submapDimensionLookup_.insert(std::make_pair(id, dims.cast<float>()));

      // index the box, without caring about orientation.
      // its just a dumb hack to allow planning latestStatefor current submap
      for (int x = min_box(0); x <= max_box(0); x += 1) {
        for (int y = min_box(1); y <= max_box(1); y += 1) {
          for (int z = min_box(2); z <= max_box(2); z += 1) {
            Eigen::Vector3i pos(x, y, z);

            // add to index
            hashTable_[pos].insert(id);

            // add pos to inverse index
            hashTableInverse_[id].insert(pos);
          }
        }
      }

      lk.unlock();

    }
*/
/*
// do not change pass by value
    void
    SubmappingInterface::doSpatialHashing(const uint64_t id, const Transformation& Tf, const std::shared_ptr<se::OccupancyMap<se::Res::Multi>> map) {

      Eigen::Vector3i min_box(1000, 1000, 1000);
      Eigen::Vector3i max_box(-1000, -1000, -1000);

      // ======== get bounding box dimensions (in map frame) ========

      auto octree_ptr = map->getOctree();
      std::cout << "hi";

      auto resolution = map->getRes();
      Eigen::Matrix4f T_KM = map->getTWM();
      Eigen::Matrix4d T_WK = Tf.T();
      Eigen::Matrix4f T_WM = T_WK.cast<float>() * T_KM;

      for (auto octant_it = se::LeavesIterator<OctreeT>(octree_ptr.get());
           octant_it != se::LeavesIterator<OctreeT>(); ++octant_it) {
        const auto octant_ptr = *octant_it;

        // first, check if octant (node or block, dont care at this stage), is completely inside current bounds
        // get the two min and max corners, and check them against bounds
        Eigen::Vector3i corner_min = octant_ptr->getCoord();
        Eigen::Vector3i corner_max =
                corner_min + Eigen::Vector3i::Constant(se::octantops::octant_to_size<OctreeT>(octant_ptr));
        bool inside_bounds = true;
        for (int i = 0; i < 3; i++) {
          // if not inside bounds
          if (corner_min(i) < min_box(i) || corner_max(i) > max_box(i)) {
            inside_bounds = false;
            break;
          }
        }
        // if octant is completely inside --> skip octant
        if (inside_bounds) continue;

        // Differentiate between block and node processing
        if (octant_ptr->isBlock()) {
          // If the leaf is a block we'll need to iterate over all voxels at the current scale
          const Eigen::Vector3i block_coord = octant_ptr->getCoord();
          const BlockType *block_ptr = static_cast<const BlockType *>(octant_ptr);
          // Find current scale of the block leaves and their size
          const int node_scale = block_ptr->getCurrentScale();
          const int node_size = 1 << node_scale;

          // iterate through voxels inside block
          for (unsigned int x = 0; x < BlockType::getSize(); x += node_size) {
            for (unsigned int y = 0; y < BlockType::getSize(); y += node_size) {
              for (unsigned int z = 0; z < BlockType::getSize(); z += node_size) {


                // if the voxel is unobserved, skip
                const auto data = block_ptr->getData();
                if (data.weight == 0) continue;

                const Eigen::Vector3i node_coord = block_coord + Eigen::Vector3i(x, y, z);
                // std::cout << "voxel_coord \n" << node_coord << "\n";


                Eigen::Vector3i voxel_corner_min = node_coord;
                Eigen::Vector3i voxel_corner_max = voxel_corner_min + Eigen::Vector3i(node_size, node_size, node_size);
                for (int i = 0; i < 3; i++) {
                  // if not inside bounds, update either max or min
                  if (voxel_corner_min(i) < min_box(i)) {
                    // std::cout << "update min \n";
                    min_box(i) = voxel_corner_min(i);
                  }
                  if (voxel_corner_max(i) > max_box(i)) {
                    // std::cout << "update max \n";
                    max_box(i) = voxel_corner_max(i);
                  }
                }

                // ... or, just check for voxel coord. not for all corners ?
                // faster, but with the adaptive res thing, could mean that the voxels are huge and
                // we are actually making huge errors, not 20 cm errors

              } // z
            } // y
          } // x
        } else { // if is node


          const auto data = static_cast<typename OctreeT::NodeType *>(octant_ptr)->getData();
          if (data.weight == 0) continue;

          const int node_size = static_cast<typename OctreeT::NodeType *>(octant_ptr)->getSize();

          const Eigen::Vector3i node_coord = octant_ptr->getCoord();

          Eigen::Vector3i node_corner_min = node_coord;
          Eigen::Vector3i node_corner_max = node_corner_min + Eigen::Vector3i(node_size, node_size, node_size);
          for (int i = 0; i < 3; i++) {
            // if not inside bounds, update either max or min
            if (node_corner_min(i) < min_box(i)) {
              // std::cout << "update min \n";
              min_box(i) = node_corner_min(i);
            }
            if (node_corner_max(i) > max_box(i)) {
              // std::cout << "update max \n";
              max_box(i) = node_corner_max(i);
            }
          }

        }
      }

      Eigen::Vector3f min_box_metres;
      Eigen::Vector3f max_box_metres;
      for (int i = 0; i < 3; i++) {
        min_box_metres(i) = min_box(i) * resolution;
        max_box_metres(i) = max_box(i) * resolution;
      }

      // now I have the bounding box in metres, wrt the map frame.
      // this frame is separated from the real world frame by: Twk*Tkm
      // so to do hashing we must transform this box to the world frame by using this transformation
      // just like I did before with the stupid hashing. but with a double transformation

      std::unique_lock<std::mutex> lk(hashTableMutex_);

      // we first need to get rid of the preliminary indexing we did when
      // creating map (we indexed a 10x10x10 box)
      if (hashTableInverse_.count(id)) { // this should always be the case
        for (const auto &pos: hashTableInverse_[id]) {
          hashTable_[pos].erase(id); // remove id from box
          //hashTableInverse_[id].erase(pos); // remove box from id
          submapDimensionLookup_.erase(id); // remve dimensions
        }
        hashTableInverse_[id].clear();
      }

      // insert map bounds in the lookup
      Eigen::Matrix<float, 6, 1> dims;
      dims << min_box_metres, max_box_metres;
      submapDimensionLookup_.insert(std::make_pair(id, dims));

      // Check first if there already is an entry in hash table. if there is, we should delete and rewrite.
      // But to keep things simple, we just ignore that case. That happens when an older map is re-integrated
      // We only redo the hashing when a loop closure is detected.

      const float side = 1.0; // hardcoded hash map box side of 1m
      const float step = 0.5 * side * sqrt(2); // this ensures full cover of submap space

      // need to take all points -> use <=
      for (float x = min_box_metres(0); x <= max_box_metres(0); x += step) {
        for (float y = min_box_metres(1); y <= max_box_metres(1); y += step) {
          for (float z = min_box_metres(2); z <= max_box_metres(2); z += step) {

            // get offset value (this pos is in map frame)
            Eigen::Vector4f pos_map(x, y, z, 1);

            // transform into world frame
            Eigen::Vector4f pos_world;
            pos_world = T_WM * pos_map;

            // floor transformed value
            Eigen::Vector3i pos_floor;
            for (int i = 0; i < 3; i++) {
              // if side is e.g. 2, a value of 4.5,4.5,4.5 is mapped to box 2,2,2
              pos_floor(i) = (int) (floor(pos_world(i) / side));
            }

            // std::cout << "   box \n " << pos_floor <<  "\n";

            // add to index
            hashTable_[pos_floor].insert(id);

            // add pos to inverse index
            hashTableInverse_[id].insert(pos_floor);

            // for (int i = 0; i < 3; i++) {
            //   if (pos_floor(i) < minbounds(i)) minbounds(i) = pos_floor(i);
            //   if (pos_floor(i) > maxbounds(i)) maxbounds(i) = pos_floor(i);
            // }
          }
        }
      }

      lk.unlock();

    }
*/
// Transform the AABB of the submap from submap frame to world frame.
    void SubmappingInterface::aabb_trasnform (Eigen::Vector3f& min_coords_W, Eigen::Vector3f& max_coords_W, 
                        Eigen::MatrixXf vert_coords_M, const Eigen::Matrix4f T_WM) {
        // Transform from Cartesian coordinates into homogeneous coordinates 
        vert_coords_M.conservativeResize(vert_coords_M.rows()+1, vert_coords_M.cols());
        vert_coords_M.row(vert_coords_M.rows()-1) = Eigen::VectorXf::Ones(vert_coords_M.cols());
        // Transform coordinates of vertices from submap frame to world frame
        Eigen::MatrixXf vert_coords_W = T_WM*vert_coords_M;
        // Initialization before search
        min_coords_W << 1e6, 1e6, 1e6;
        max_coords_W << -1e6, -1e6, -1e6;
        // Get the maximal and minimal coordinates of the AABB in world frame
        for (int i = 0; i < vert_coords_W.rows() - 1; i++) {
            for (int j = 0; j < vert_coords_W.cols(); j++) {
                if (vert_coords_W(i, j) < min_coords_W(i)) {min_coords_W(i) = vert_coords_W(i, j);}
                if (vert_coords_W(i, j) > max_coords_W(i)) {max_coords_W(i) = vert_coords_W(i, j);}
            }
        }
    }

// do not change pass by value
    void 
    SubmappingInterface::SpatialHashing(const uint64_t id, const Transformation Tf, const std::shared_ptr<se::OccupancyMap<se::Res::Multi>> map) {

      // Get coordinates of 8 vertices of AABB in submap frame K.
      Eigen::Matrix<float, 3, 2> aabb_bounds_M;
      aabb_bounds_M.col(0) = map->aabbMin();
      //std::cout << "The minimal coordinates of the submap is: " << aabb_bounds_M.col(0) << std::endl;
      aabb_bounds_M.col(1) = map->aabbMax();
      //std::cout << "The maximal coordinates of the submap is: " << aabb_bounds_M.col(1) << std::endl;
      Eigen::Matrix<float, 3, 8> vert_coords_M;
      for (int i = 0; i < 3; i++)
      {
          for (int j = 0; j < pow(2, i); j++)
          {
              vert_coords_M.block(i, pow(2, 3 - i)*j, 1, pow(2, 2 - i)) << Eigen::MatrixXf::Constant(1, pow(2, 2 - i), aabb_bounds_M(i, 0));
              vert_coords_M.block(i, pow(2, 3 - i)*j + pow(2, 2 - i), 1, pow(2, 2 - i)) << Eigen::MatrixXf::Constant(1, pow(2, 2 - i), aabb_bounds_M(i, 1));
          }
      }
      //std::cout << "AABB's vertices coordinates in submap frame: " << std::endl << vert_coords_M << std::endl;

      //std::unique_lock<std::mutex> lk(hashTableMutex_);

      // Remove previous hashing.
        for (const auto &ind: hashTableInverse_[id]) {
          hashTable_[ind].erase(id); // remove id from box
          //submapDimensionLookup_.erase(id); // remve dimensions
        }

      hashTableInverse_[id].clear();

      // Transformation between frames.
      //Eigen::Matrix4f T_KM = map->getTWM();
      //std::cout << "T_KM given by getTWM() is " << std::endl << T_KM << std::endl;
      //Eigen::Matrix4d T_WK = Tf.T();
      //std::cout << "T_WK given by Tf.T() is " << std::endl << T_WK << std::endl;
      //Eigen::Matrix4f T_WM = T_WK.cast<float>() * T_KM;

      // Transform from submap frame into world frame.
      Eigen::Vector3f min_coords_W, max_coords_W;
      SubmappingInterface::aabb_trasnform (min_coords_W, max_coords_W, vert_coords_M, Tf.T().cast<float>());
      //std::cout << "AABB's minimal bounds in world frame: " << std::endl << min_coords_W << std::endl;
      //std::cout << "AABB's maximal bounds in world frame: " << std::endl << max_coords_W << std::endl;

      // Define virtual hash grid
      //const float cell_size = 2.0;
      // Get the start indices and end indices of the submap' AABB
      Eigen::Vector3i min_ind, max_ind;
      for (int i = 0; i < 3; i++)
      {
          min_ind(i) = floor(min_coords_W(i)/hash_cell_size);
          max_ind(i) = floor(max_coords_W(i)/hash_cell_size);
      }
      //std::cout << "AABB's minimal indices in hash grid: " << std::endl << min_ind << std::endl;
      //std::cout << "AABB's maximal indices in hash grid: " << std::endl << max_ind << std::endl;
      
      // Do bidirectional hashing
      for (int x = min_ind(0); x <= max_ind(0); x++)
      {
          for (int y = min_ind(1); y <= max_ind(1); y++)
          {
              for (int z = min_ind(2); z <= max_ind(2); z++)
              {
                  Eigen::Vector3i cell_ind; 
                  cell_ind << x, y, z;

                  //std::cout << "The hash cell id is " << cell_ind.transpose() << " and the submap id is " << id << std::endl;

                  hashTable_[cell_ind].insert(id);
                  //allSubmapIDs_.insert(id);
                  hashTableInverse_[id].insert(cell_ind);
              }
          }
      }
      //lk.unlock();
    }

}