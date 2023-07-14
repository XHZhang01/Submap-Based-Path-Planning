//
// Created by boche on 10/7/22.
//

#ifndef INCLUDE_OKVIS_SUBMAPPINGINTERFACE_HPP
#define INCLUDE_OKVIS_SUBMAPPINGINTERFACE_HPP

#include <thread>
#include <vector>
#include <utility>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <chrono>
#include <condition_variable>
#include <functional>
#include "okvis/FrameTypedefs.hpp"
#include "okvis/Measurements.hpp"
#include "okvis/ViInterface.hpp"
#include <okvis/ThreadedSlam.hpp>
#include "okvis/ViParametersReader.hpp"
#include "okvis/ceres/ImuError.hpp"
#include "okvis/kinematics/Transformation.hpp"
#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <se/supereight.hpp>
#include <iostream>
#include <fstream>
#include <set>
#include <se/planning/path_planner.hpp>
#include <okvis/config_mapping.hpp>

#include <random>
#include <algorithm>
#include <Eigen/StdVector>

// Some convenient typedefs
typedef se::Image<float> DepthFrame;
typedef okvis::CameraMeasurement CameraMeasurement;
typedef okvis::threadsafe::Queue<CameraMeasurement> DepthFrameQueue;
// --- Leica Definitions Start
typedef okvis::LidarMeasurement LidarMeasurement;
typedef okvis::threadsafe::Queue<LidarMeasurement> LidarMeasurementQueue;
// --- Leica Definitions End
typedef okvis::StateId StateId;
typedef okvis::kinematics::Transformation Transformation;
typedef okvis::TrackingState TrackingState;
typedef okvis::AlignedVector<okvis::State> StateVector;
typedef okvis::State State;
typedef se::Octree<se::Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::Off>, se::Res::Multi, 8> OctreeT;
typedef typename OctreeT::BlockType BlockType;


namespace okvis {
/**
 * @brief Extended okvis update: contains all the things we need for depth integration.
 *
 */
    struct OkvisUpdate {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        State latestState;
        StateVector keyframeStates;
        okvis::Time timestamp;
        bool isKeyframe;
        uint64_t currentKeyframe;
        bool loop_closure;
        TrackingState trackingState;
        std::set<StateId> affectedStates;
        std::shared_ptr<const okvis::AlignedMap<StateId, State>> updatedStates; 

        OkvisUpdate(const State &latestState = State(),
                    const StateVector &keyframeStates = StateVector(),
                    const okvis::Time &timestamp = okvis::Time(),
                    const bool isKeyframe = false,
                    const uint64_t currentKeyframe = 1,
                    const bool &loop_closure = false, 
                    const TrackingState &trackingState = TrackingState(),
                    const std::set<StateId> affectedStates = std::set<StateId>(),
                    std::shared_ptr<const okvis::AlignedMap<StateId, State>> updatedStates = std::make_shared<const okvis::AlignedMap<StateId, State>>(okvis::AlignedMap<StateId, State>()))
                : latestState(latestState), keyframeStates(keyframeStates),
                  timestamp(timestamp), isKeyframe(isKeyframe),
                  currentKeyframe(currentKeyframe), loop_closure(loop_closure),
                  trackingState(trackingState), affectedStates(affectedStates), updatedStates(updatedStates) {};
        
        OkvisUpdate& operator=(OkvisUpdate& newElem){
            latestState = newElem.latestState;
            keyframeStates = newElem.keyframeStates;
            timestamp = newElem.timestamp;
            isKeyframe = newElem.isKeyframe;
            currentKeyframe = newElem.currentKeyframe;
            loop_closure = newElem.loop_closure;
            trackingState = newElem.trackingState;
            affectedStates = newElem.affectedStates;
            updatedStates = newElem.updatedStates;
            return *this;
        }
    };

    typedef okvis::threadsafe::Queue<OkvisUpdate, Eigen::aligned_allocator<OkvisUpdate>> StateUpdatesQueue;

/**
 * @brief Contains essential data about a keyframe: its Id and transformation.
 *
 */
    struct KeyframeData {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        uint64_t id;
        Transformation T_WS;

        KeyframeData(const uint64_t &id = 0,
                     const Transformation &T_WS = Transformation::Identity())
                : id(id), T_WS(T_WS) {};
    };

    typedef std::map<uint64_t, KeyframeData, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, KeyframeData>>> KeyFrameDataMap;

/**
 * @brief Contains the data required for a single supereight map integration
 * step. each seframe contains the entire list of keyframes with updated poses
 *
 */
    struct SupereightFrames {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        //DepthFrame depthFrame;
        std::vector<std::pair<Eigen::Matrix4f, Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f, Eigen::Vector3f>> > vecRayMeasurements;
        std::vector<std::pair<Eigen::Matrix4f, cv::Mat>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f, cv::Mat>>> vecDepthFrames;
        uint64_t keyFrameId; // id of current kf
        KeyFrameDataMap kfData_;
        bool loop_closure;
        bool endSubmap;
        unsigned int stateCounter_;



        SupereightFrames(const uint64_t &keyframeId = 0,
                        const KeyFrameDataMap &keyFrameDataMap = KeyFrameDataMap{},
                        const bool &loop_closure = false,
                        std::vector<std::pair<Eigen::Matrix4f, Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f, Eigen::Vector3f>> >* vecRayMeasurementsVal = nullptr,
                        std::vector<std::pair<Eigen::Matrix4f, cv::Mat>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f, cv::Mat>> >* vecDepthFramesVal = nullptr)
                          : keyFrameId(keyframeId), kfData_(keyFrameDataMap), loop_closure(loop_closure),
                            endSubmap(false), stateCounter_(0) {

                        if(vecRayMeasurementsVal){
                                for(size_t i = 0; i < vecRayMeasurementsVal->size(); i++){
                                        vecRayMeasurements.push_back(vecRayMeasurementsVal->at(i));
                                }
                        }

                        if(vecDepthFramesVal){
                                for(size_t i = 0; i < vecDepthFramesVal->size(); i++){
                                        vecDepthFrames.push_back(vecDepthFramesVal->at(i));
                                }
                        }

                  };
    };

    struct SubmapAlignmentTerm{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        uint64_t frame_A_id; ///< ID of keyframe A (submap to be referred to (previous submap))
        uint64_t frame_B_id; ///< ID of keyframe B  (current new submap)
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> pointCloud_B; ///< Point cloud with points in keyframe B that are aligned to submap in frame A

        SubmapAlignmentTerm() : frame_A_id(0), frame_B_id(0)
        {
        };

        SubmapAlignmentTerm(const uint64_t& frame_A_id, const uint64_t& frame_B_id)
          : frame_A_id(frame_A_id), frame_B_id(frame_B_id)
        {
        };

    };


    typedef okvis::threadsafe::Queue<SupereightFrames, Eigen::aligned_allocator<SupereightFrames>> SupereightFrameQueue;

    typedef std::list<std::shared_ptr<se::OccupancyMap<se::Res::Multi>>> SubmapList;

    typedef std::function<void(AlignedUnorderedMap<uint64_t, Transformation>)> submapMeshesCallback;
    typedef std::function<void(AlignedUnorderedMap<uint64_t, Transformation>,
                               std::unordered_map<uint64_t, std::shared_ptr<se::OccupancyMap<se::Res::Multi>>>)> submapCallback;
    typedef std::function<void(const State&,
                               const AlignedUnorderedMap<uint64_t, Transformation>&,
                               const std::unordered_map<uint64_t, std::shared_ptr<se::OccupancyMap<se::Res::Multi>>>&)> fieldCallback;
    typedef std::function<void(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>)> pathPlannerCallback;

    class SubmappingInterface {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SubmappingInterface() = delete;

        /**
         * @brief      Constructs a new instance.
         *
         * @param[in]  cameraConfig Pinhole camera configuration.
         * @param[in]  mapConfig Map configuration.
         * @param[in]  dataConfig Occupancy mapping configuration.
         * @param[in]  T_SC Homogenous transformation from sensor to depth camera.
         * @param[in]  mapConfig Map configuration.
         * @param[in]  meshesPath  Directory where we store map meshes.
         */
        SubmappingInterface(
                const se::MapConfig &mapConfig,
                const se::OccupancyDataConfig &dataConfig,
                const se::SubMapConfig &submapConfig,
                std::pair<Eigen::Matrix4d, se::LeicaLidar>* lidarConfig = nullptr,
                std::pair<Eigen::Matrix4d, se::PinholeCamera>* cameraConfig = nullptr)
                : mapConfig_(mapConfig), dataConfig_(dataConfig), meshesPath_(submapConfig.resultsDirectory),
                  kfThreshold_(submapConfig.submapKfThreshold), stateThreshold_(submapConfig.submapStateThreshold),
                  lidarSensor_(nullptr), cameraSensor_(nullptr),
                  sensorMeasurementDownsampling_(submapConfig.sensorMeasurementDownsampling),
                  depthImageResDownsamplingRatio_(submapConfig.depthImageResDownsampling){

          blocking_ = false;
          
          //Quite a hacky way of doing this. Problem is that as the variable the pointers point at run afterwards 
          //in a different thread, they are getting deleted once we get out of scope. If anyone has got a better approach/proposal
          // I am quite open as to improve this initialization

          if(lidarConfig){
                lidarSensor_ = new std::pair<Transformation, se::LeicaLidar>(std::make_pair(Transformation(lidarConfig->first), lidarConfig->second));
          }

          if(cameraConfig){
            cameraSensor_ = new std::pair<Transformation, se::PinholeCamera>(std::make_pair(Transformation(cameraConfig->first), cameraConfig->second));
          }
  
        };

        /**
         * @brief      Destroys the object.
         */
        ~SubmappingInterface() {
          // Shutdown all the Queues.
          depthMeasurements_.Shutdown();
          lidarMeasurements_.Shutdown();
          stateUpdates_.Shutdown();
          supereightFrames_.Shutdown();

          // Wait for threads
          if(dataIntegration_.joinable()) {
            dataIntegration_.join();
          }

          LOG(INFO) << "Finished data integration thread";

          if(submapIntegration_.joinable()){
                submapIntegration_.join();
          }

          LOG(INFO) << "Finished submap integration thread";

        };

        /**
         * @brief     Adds a lidar measurement to the measurement Queue.
         * @param[in] stamp     The timestamp.
         * @param[in] ray       The lidar measurement
         *
         * @return    True if successful.
         */
        bool addLidarMeasurement(const okvis::Time &stamp, const Eigen::Vector3d &ray);

        /**
         * @brief Adds a depth image to the measurement Queue
         * @param[in] stamp The timestamp
         * @param[in] depthImage The depth image
         * 
         * @return True if successful
        */
        bool addDepthMeasurement(const okvis::Time &stamp, const cv::Mat &depthImage);

        /**
         * @brief      Starts the processing and data prepatation threads.
         *
         * @return     True when successful.
         */
        bool start();

        /**
         * @brief      Stores the state and keyframe updates provided by OKVIS
         *
         * @param[in]  latestState          The current OKVIS state
         * @param[in]  latestTrackingState  The current tracking state
         * @param[in]  keyframeStates       The state of the updated Keyframes
         *
         * @return     True when successful
         */
        bool stateUpdateCallback(const State &latestState,
                                 const TrackingState &latestTrackingState,
                                 std::shared_ptr<const okvis::AlignedMap<StateId, State>> updatedStates);
                                 

        /**
         * @brief      Gets the size of the to-be-processed supereight frames.
         *
         * @return     The supereight queue size.
         */
        size_t getSupereightQueueSize() { return supereightFrames_.Size(); };

        /**
         * @brief      Set blocking/not blocking mode.
         *
         */
        void setBlocking(bool blocking) {
          blocking_ = blocking;
        }

        /**
         * @brief      Main function of the processing thread. It integrates the
         * assembled supereigh frames (i.e. depth/lidar ray + pose) and creates a new submap
         * when required
         */
        void processSupereightFrames();

        /**
         * @brief       Checks if required data (LiDAR measurements + OKVIS estimates) is available and prepares for
         * processing / integration into the map
         *
         * @return      True, if data available. False otherwise.
         */
        bool checkForAvailableData();

        /**
         * @brief Function which updates the supereight frame with the new poses of the keyframes and metadata regarding loop closures and keyframeID
         * @param[in] propagatedStateUpdate okvis update with all the new information of the current state which has been processed
        */
        void frameUpdater(okvis::OkvisUpdate& propagatedStateUpdate);

        /**
         * @brief Function which updates the Supereight frame with the new measurement depth data.
         *
         * @tparam Either LidarSensorReadings or CameraData
         * @param[in] depthMeasurement Either Lidar depth data or Depth camera data
         * @param[in] T_WD The depth sensor pose wrt world coordinates
        */
        template<typename DepthSource>
        void depthFrameUpdater(Measurement<DepthSource>& depthMeasurement, Eigen::Matrix4f& T_WD);


        void collectInfo(){
            std::cout << "There are " + std::to_string(stateUpdates_.Size()) + " messages in the state queue" << std::endl;
            std::cout << "There are " + std::to_string(supereightFrames_.Size()) + " messages in the supereight queue" << std::endl;
            //std::cout << "There are " + std::to_string(lidarMeasurements_.Size()) + " messages in the lidar measurements queue" << std::endl;
            std::cout << "There are " + std::to_string(depthMeasurements_.Size()) + " messages in the depth camera measurements queue" << std::endl;
            CameraMeasurement oldestDepthImage;
            OkvisUpdate toPopState;
            OkvisUpdate newestState;
            if(stateUpdates_.getCopyOfFront(&toPopState)){
              stateUpdates_.getCopyOfBack(&newestState);
              std::cout << "Earliest timestamp from the states is " << toPopState.timestamp << std::endl;
              std::cout << "Most recent timestamp from the states is " << newestState.timestamp << std::endl;
            }

            if(depthMeasurements_.getCopyOfFront(&oldestDepthImage)) {
              std::cout << "Earliest timestamp from the depth image is " << oldestDepthImage.timeStamp << std::endl;
            }

        }

/**
   * @brief      Set function that handles submaps visualization (meshes version)
   *
   */
        void setSubmapMeshesCallback(
                const submapMeshesCallback &submapMeshesCallback) { submapMeshesCallback_ = submapMeshesCallback; }

/**
   * @brief      Set function that handles submaps visualization (blocks version)
   *
   */
        void setSubmapCallback(const submapCallback &submapCallback) { submapCallback_ = submapCallback; }

        void setFieldSliceCallback(const fieldCallback &fieldCallback) { fieldCallback_ = fieldCallback; }

/**
   * @brief      Launch visualization thread.
   *
   */
        void publishSubmaps(AlignedUnorderedMap<uint64_t, Transformation> submapPoses,
                            std::unordered_map<uint64_t, std::shared_ptr<se::OccupancyMap<se::Res::Multi>>> submaps);

        /**
         * @brief Function which does the saving of submaps into meshes
         * @param[in] kfId the Id of the submaps keyframe we are going to store
        */
        void saveSubmap(uint64_t kfId);

        void updateSEKeyFrameData(KeyFrameDataMap& seKFData, KeyFrameDataMap& newKFData){
            for(auto it : newKFData) seKFData[it.first] = it.second;
            return;
        }

        void setT_BS(const okvis::kinematics::Transformation& T_BS);


// to hash a 3 int eigen vector
        struct SpatialHasher {
            std::size_t operator()(const Eigen::Vector3i &a) const {
              std::size_t h = 0;

              // taken from matthias teschner 2003 collision
              const int p1 = 73856093;
              const int p2 = 19349663;
              const int p3 = 83492791;
              h = a[0] * p1 ^ a[1] * p2 ^ a[2] * p3;

              return h;
            }
        };

        // To access maps
        std::unordered_map<uint64_t, std::shared_ptr<se::OccupancyMap<se::Res::Multi>>> submapLookup_; // use this to access submaps (index,submap)
        AlignedUnorderedMap<uint64_t, Transformation> submapPoseLookup_; // use this to access submap poses (index,pose in camera frame) - only submap keyframes
        AlignedUnorderedMap<uint64_t, Eigen::Matrix<float, 6, 1>> submapDimensionLookup_; // use this when reindexing maps on loop closures (index,dims)
        // spatial hash maps: side x side x side boxes
        //std::unordered_map<Eigen::Vector3i, std::unordered_set<uint64_t>, SpatialHasher> hashTable_; // a hash table for quick submap access (box coord, list of indices)
        //std::unordered_map<uint64_t, std::unordered_set<Eigen::Vector3i, SpatialHasher>> hashTableInverse_; // inverse access hash table (index, list of box coords)
        se::planning::HashUnorderedMap<uint64_t> hashTable_; // a hash table for quick submap access (box coord, list of indices)
        se::planning::InvHashUnorderedMap<uint64_t> hashTableInverse_; // inverse access hash table (index, list of box coords)
        const float hash_cell_size = 2.0; // dimension of hash cell
        /**
         * @brief Function to know if there are still messages to be processed or not
         * @return true if finished false if there is still data to be processed
        */
        bool finishedIntegrating();
        
        /**
         * @brief Function to check if the integration of the interface has finished
        */
        bool isFinished(){
            std::lock_guard<std::mutex> l(finishMutex_);
            return isFinished_;
        }

        /**
         * @brief Function that sets the finished boolean to true
        */
        void setFinished(){
            std::lock_guard<std::mutex> l(finishMutex_);
            isFinished_ = true;
            return;
        }

        /// @brief Set the callback for tightly coupling alignment terms to OKVIS
        /// @param imuCallback The IMU callback to register.
        typedef std::function<
                bool(const se::OccupancyMap<se::Res::Multi>* submap_ptr, const uint64_t& frame_A_id, const uint64_t& frame_B_id, std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& pointCloud)> AlignCallback;

        void setAlignCallback(const AlignCallback& alignCallback) {
          alignCallback_ = alignCallback;
        }

        void setPathPlannerCallback(const pathPlannerCallback& plannerCallback){
            pathPlannerCallback_ = plannerCallback;
        }

        //DEBUGGING POSES
        void addGtPoses(okvis::Time t, okvis::kinematics::Transformation transform){
            std::lock_guard<std::mutex> l(gtDebug_);
            gtPoses_[t.toNSec()] = transform;
        }

        bool getGtPoses(okvis::Time t, okvis::kinematics::Transformation &T_WS){
            std::lock_guard<std::mutex> l(gtDebug_);
            
            if(gtPoses_.find(t.toNSec()) != gtPoses_.end()){
                T_WS = gtPoses_[t.toNSec()];
                return true;
            }
            return false;
        }

        void setUseGtPoses(bool flag){
            useGtPoses_ = flag;
            return;
        }

        void generatePointCloud(const Eigen::Matrix4f& T_WS, const DepthFrame& dImage, const se::PinholeCamera& sensor, int counter);

        /**
         * @brief Function that stores raw pointclouds from lidar measurements
         * 
         * @param[in] T_WD Is the transform between lidar sensor and world frame
         * @param[in] 
        */
        void generatePointCloudFromLidar(const std::vector<std::pair<Eigen::Matrix4f, Eigen::Vector3f>,
                Eigen::aligned_allocator<std::pair<Eigen::Matrix4f, Eigen::Vector3f>>>& lidarMeas, int counter);

	/**
	 * @brief Set the goal pose (position and yaw) for the path planner.
	 * @param[in] A relative goal pose, that is the goal body frame (Bg)
	 * expressed in the current body frame (B). The robot body frame (B) is
	 * assumed to be x-forward, z-up.
	 */
	void setGoal(const Eigen::Matrix4d& T_BBg);

    private:

        /**
         * @brief Loop that performs the supereight frame data collection and integrates it
        */
        void integrationLoop();

        
        /**
         * @brief      Given time stamp of depth frame, propagates latest okvis update to get pose estimate. It also gets from the states updates queue:
         * vector of updated keyframes, latest keyframe id, loop closure status.
         *
         * @param[in]  finalTimestamp  Timestamp of the depth frame.
         * @param[out]  T_WC  Predicted depth frame pose w.r.t. world.
         * @param[in]   isCamera Is this prediction for the camera sensor or for the lidar sensor
         *
         * @return     The depth frame as a supereight Image
         */
        // ToDo: adjust documentation
        bool predict(const okvis::Time &finalTimestamp,
                     Transformation &T_WC,
                     bool isCamera = true);

        /**
         * @brief   Re assign spatial hash table for a given map.
         *
         * @param[in]  id  Id of the map.
         * @param[in]  Tf  Pose of the map
         * @param[in]  map  Pointer to the map.
         *
         */
        //void redoSpatialHashing(const uint64_t id, const Transformation& Tf, const std::shared_ptr<se::OccupancyMap<se::Res::Multi>> map);

        /**
         * @brief   Pre-index new map as soon as we start integrating.
         *
         * @param[in]  id  Id of the map.
         * @param[in]  pos_kf Position of the Keyframe related to the map.
         *
         */
        //void doPrelimSpatialHashing(const uint64_t id, const Eigen::Vector3d& pos_kf);
        
        /**
         * @brief Convert cv::Mat images into se::Image which is the input for Supereight
         * @param[in] inputDepth cv::Mat depth image data which is going to be converted to se::Image<float> for Supereight
         *         
         * @return DepthFrame which is a typedef from se::Image<float>
         */ 
        DepthFrame depthMat2Image(const cv::Mat &inputDepth);

        /**
         * @brief   Index new map when it's done being integrated.
         *
         * @param[in]  id  Id of the map.
         * @param[in]  Tf  Pose of the map
         * @param[in]  map  Pointer to the map.
         *
         */
        //void doSpatialHashing(const uint64_t id, const Transformation& Tf, const std::shared_ptr<se::OccupancyMap<se::Res::Multi>> map);

        /**
         * @brief   Transform the submap's AABB from submap frame to world frame and get the new bounds.
         *
         * @param[in]  min_coords_W  Vector for storage of AABB's minimal bounds in world frame.
         * @param[in]  max_coords_W  Vector for storage of AABB's maximal bounds in world frame.
         * @param[in]  vert_coords_M  Coordinates of AABB's 8 vertices in submap frame.
         * @param[in]  T_WM  Transformation from submap frame to world frame.
         *
         */        
        void aabb_trasnform (Eigen::Vector3f& min_coords_W, Eigen::Vector3f& max_coords_W, 
                            Eigen::MatrixXf vert_coords_M, const Eigen::Matrix4f T_WM);

        /**
         * @brief   Spatial hashing for the current map.
         *
         * @param[in]  id  Id of the map.
         * @param[in]  Tf  Pose of the map
         * @param[in]  map  Pointer to the map.
         *
         */
        void SpatialHashing(const uint64_t id, const Transformation Tf, const std::shared_ptr<se::OccupancyMap<se::Res::Multi>> map);
  
        //se::PinholeCamera sensor_;      ///< Depth sensor used in supereight
        
        se::MapConfig mapConfig_;       ///< Supereight Map config
        se::OccupancyDataConfig dataConfig_; ///< Supereight Data config
        std::string meshesPath_;  ///< Path to save the meshes
        
        DepthFrameQueue
                depthMeasurements_; ///< Queue with the buffered Depth measurements
        LidarMeasurementQueue
                lidarMeasurements_; ///< Queue with buffered LiDAR Measurements
        StateUpdatesQueue
                stateUpdates_; ///< Queue containing all the state updates from okvis
        SupereightFrameQueue
                supereightFrames_; ///< Queue with the s8 frames (i.e. poses and depth
        
        std::mutex hashTableMutex_; // either dohashing or redohashing

        std::mutex finishMutex_; // mutex for checking if the integration of data has finished
        std::mutex gtDebug_;
        AlignedUnorderedMap<uint64_t, Transformation> gtPoses_;
        bool useGtPoses_ = false;

        // callbacks
        submapMeshesCallback submapMeshesCallback_; // to visualize in Publisher
        submapCallback submapCallback_; // to visualize in Publisher
        fieldCallback fieldCallback_; // to visualize in Publisher
        pathPlannerCallback pathPlannerCallback_;

        bool blocking_;

        // We use this to store the active keyframe in predict(), to prepare the supereightframe
        // warning: do not use anywhere else
        struct LatestKeyframe {
            uint64_t Id;
            bool started;

            LatestKeyframe() : Id(1), started(false) {};
        } latestKeyframe;

        
        
        //Keyframe threshold to generate a new map
        unsigned int kfThreshold_;
        std::set<int> kfCounter_;
        
        //Number of states processed to send as a chunk of submap
        unsigned int stateThreshold_;

        // Need this to extract okvis estimates at abitrary timestamps
        okvis::Trajectory propagatedStates;

        uint32_t integration_counter_ = 0;
        uint32_t image_counter_ = 0;

        std::set<StateId> keyframes_;

        /// Thread which is in charge of doing the submap integration when data is available
        std::thread submapIntegration_;

        std::thread dataIntegration_;

        /// Attribute with the value of the latest KeyFrame which has been processed. Initial value is -1 to ensure in the first iteration it is updated
        uint64_t prevKeyframeId_ = -1;

        SupereightFrames* frame_ = nullptr;

        bool isFinished_ = false;

        okvis::OkvisUpdate lastIntegratedUpdate_;

        size_t lidarCounter_ = 0;

        size_t depthQueueSize_ = 0;
        
        //Depth sensors which can be used in the SubmappingInterface
        std::pair<Transformation, se::LeicaLidar>* lidarSensor_; ///< Lidar sensor used in supereight ray extensionimuConsume
        std::pair<Transformation, se::PinholeCamera>* cameraSensor_;

        const int sensorMeasurementDownsampling_;

        int sensorMeasurementDownsamplingCounter_ = 0;

        const int depthImageResDownsamplingRatio_;

        // Callback for submap alignment
        AlignCallback alignCallback_;

        kinematics::Transformation T_BS_;
        kinematics::Transformation T_SB_;

        // Path planner goal.
        bool has_goal_ = false;
        Eigen::Matrix4d goal_T_BBg_;
        std::mutex goal_mtx_;

    };
} // namespace okvis
#endif //INCLUDE_OKVIS_SUBMAPPINGINTERFACE_HPP
