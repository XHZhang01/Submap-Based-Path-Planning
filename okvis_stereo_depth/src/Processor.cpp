#include <okvis/Processor.hpp>

namespace srl {

Processor::Processor(okvis::ViParameters& parameters, 
                    std::string dBowDir,
                    const se::MapConfig &mapConfig,
                    const se::OccupancyDataConfig &dataConfig,
                    const se::SubMapConfig &submapConfig,
                    std::pair<Eigen::Matrix4d, se::LeicaLidar>* lidarConfig,
                    std::pair<Eigen::Matrix4d, se::PinholeCamera>* cameraConfig)
: depthProcessor_(parameters, dBowDir),
  slam_(parameters, dBowDir),
  se_interface_(mapConfig, dataConfig, submapConfig, lidarConfig, cameraConfig) {
  // Connect the SLAM optimized graph callback to supereight.
  slam_.setOptimisedGraphCallback(std::bind(
    &Processor::internalOptimizedGraphCallback,
    this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4
  ));

  // Connect the depth prediction callback to supereight.
  depthProcessor_.setDepthImageCallback(std::bind(
    &okvis::SubmappingInterface::addDepthMeasurement, 
    &se_interface_, 
    std::placeholders::_1, std::placeholders::_2)
  );  

  // Launch supereight.
  se_interface_.start();
}

void Processor::display2(cv::Mat& matches, cv::Mat& overhead, cv::Mat& image, cv::Mat& stereoDepth) {
  slam_.display(matches, overhead);
  depthProcessor_.display(image, stereoDepth);
}

void Processor::collectInfo() {
  se_interface_.collectInfo();
}

void Processor::finish() {
  slam_.stopThreading();
  se_interface_.finishedIntegrating();
}

void Processor::setBlocking(bool blocking) {
  slam_.setBlocking(blocking);
  depthProcessor_.setBlocking(blocking);
  se_interface_.setBlocking(blocking);
}

void Processor::setT_BS(const okvis::kinematics::Transformation& T_BS) {
  se_interface_.setT_BS(T_BS);
}

bool Processor::processFrame() {
  return slam_.processFrame();
}

void Processor::setSubmapCallback(const okvis::submapCallback &callback) {
  se_interface_.setSubmapCallback(callback);
}

void Processor::setFieldSliceCallback(const okvis::fieldCallback &callback) {
  se_interface_.setFieldSliceCallback(callback);
}

void Processor::setOptimizedGraphCallback(const okvis::ViInterface::OptimisedGraphCallback &callback) {
  optimizedGraphCallback_ = callback;
}

void Processor::internalOptimizedGraphCallback(
  const okvis::State &state, 
  const okvis::TrackingState &trackingState,
  std::shared_ptr<const okvis::AlignedMap<okvis::StateId, okvis::State>> alignedMapPtr,
  std::shared_ptr<const okvis::MapPointVector> mapPointVectorPtr) {

  // Update supereight state.
  if(state.id.value() > 0) {
    se_interface_.stateUpdateCallback(state, trackingState, alignedMapPtr);
  }

  // If defined, call external optimized graph callback.
  if(optimizedGraphCallback_) {
    optimizedGraphCallback_(state, trackingState, alignedMapPtr, mapPointVectorPtr);
  }
}

bool Processor::addImages(const okvis::Time &stamp, const std::vector<cv::Mat> &images) {
  bool slamSuccess = slam_.addImages(stamp, images);
  bool depthSuccess = depthProcessor_.addImages(stamp, images);
  return slamSuccess && depthSuccess;
}

bool Processor::addImuMeasurement(const okvis::Time &stamp, const Eigen::Vector3d &alpha, const Eigen::Vector3d &omega) {
  return slam_.addImuMeasurement(stamp, alpha, omega);
}

}
