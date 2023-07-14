#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <okvis/timing/Timer.hpp>

#include <okvis/Stereo2DepthProcessor.hpp>
#include <okvis/utils.hpp>

namespace srl {

Stereo2DepthProcessor::Stereo2DepthProcessor(
  okvis::ViParameters &parameters,
  std::string modelDir,
  double visMaxDepth) 
  : visMaxDepth_(visMaxDepth) { 
  if(parameters.nCameraSystem.numCameras() != 2) {
    LOG(ERROR) << "Stereo2Depth prediction requires exactly 2 cameras";
    return;
  }
  // Set internal camera parameters for disparity -> depth conversion.
  Eigen::VectorXd intrinsics;
  parameters.nCameraSystem.cameraGeometry(0)->getIntrinsics(intrinsics);
  focalLength_ = intrinsics(0);  // left camera focal length
  baseline_ = std::abs(parameters.nCameraSystem.T_SC(0)->r()(0) - parameters.nCameraSystem.T_SC(1)->r()(0));

  // Load depth model from traced model file.
  try {
    depthModel_ = torch::jit::load(modelDir + "/depth-model.pt", torch::kCUDA);

    // Warm up forward pass for GPU.
    LOG(INFO) << "Warming up GPU ...";
    torch::Tensor leftTensor = torch::zeros({480, 640}).to(torch::kU8).to(torch::kCUDA);;
    torch::Tensor rightTensor = torch::zeros({480, 640}).to(torch::kU8).to(torch::kCUDA);
    for(int i = 0; i < 10; i++) {
      std::cout << "GPU warmup iteration " << i << std::endl;
      depthModel_.forward({leftTensor, rightTensor});
    }
    LOG(INFO) << "Done warming up GPU.";
  }
  catch (const c10::Error& e) {
    LOG(ERROR) << "Error loading the depth model from depth-model.pt";
    return;
  }

  // Starting depth processing thread.
  shutdown_ = false;
  processingThread_ = std::thread(&Stereo2DepthProcessor::processing, this);
}

Stereo2DepthProcessor::~Stereo2DepthProcessor() {
  cameraMeasurementsQueue_.Shutdown();
  visualisationsQueue_.Shutdown();
  shutdown_ = true;
  processingThread_.join();
}

void Stereo2DepthProcessor::setBlocking(bool blocking) {
  blocking_ = blocking;
  LOG(WARNING) << "setBlocking(" << (blocking ? "true" : "false") 
               << ") not yet implemented for Stereo2DepthProcessor";
}

void Stereo2DepthProcessor::display(cv::Mat& image, cv::Mat& stereoDepth) {
  if(visualisationsQueue_.Empty()) {
    return;
  }
  VisualizationData vis_data;
  if(visualisationsQueue_.PopNonBlocking(&vis_data)) {
    image = vis_data.frame.measurement.leftImage;
    stereoDepth = vis_data.depthImage;
  }
}

bool Stereo2DepthProcessor::addImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) {
  if(images.size() != 2) {
    LOG(WARNING) << "invalid vector of images, expected stereo image pair";
    return false;
  }

  StereoMeasurement frame;
  frame.measurement.leftImage = images[0];
  frame.measurement.rightImage = images[1];
  frame.timeStamp = stamp;

  if(blocking_){
    cameraMeasurementsQueue_.PushBlockingIfFull(frame, 1);
    return true;
  }
  else{
    const int queue_size = 10;
    if(cameraMeasurementsQueue_.PushNonBlockingDroppingIfFull(frame, queue_size)) {
      LOG(WARNING) <<  "stereo depth frame drop";
      return false;
    }
    else{
      return true;
    }
  }
}

void Stereo2DepthProcessor::processStereoNetwork(StereoMeasurement& frame){

  okvis::TimerSwitchable t1("Stereo Depth 1 - cv::Mat to Tensor");
  auto leftTensor = cvMatToTensor(frame.measurement.leftImage).to(torch::kCUDA);
  auto rightTensor = cvMatToTensor(frame.measurement.rightImage).to(torch::kCUDA);
  t1.stop();

  okvis::TimerSwitchable t2("Stereo Depth 2 - Model inference");
  torch::Tensor outputDisparity = depthModel_.forward({leftTensor, rightTensor}).toTensor();
  outputDisparity = outputDisparity.squeeze();
  torch::Tensor outputDepth = (focalLength_ * baseline_) / outputDisparity;
  t2.stop();

  okvis::TimerSwitchable t3("Stereo Depth 3 - Tensor to cv::Mat & Callback");
  cv::Mat outputDepthMat = tensorToCvMatFloat(outputDepth.to(torch::kFloat).detach().cpu());
  outputDepthMat *= 1000.0;
  if(depthCallback_) {
    depthCallback_(frame.timeStamp, outputDepthMat);
  }
  t3.stop();

  okvis::TimerSwitchable t4("Stereo Depth 4 - Draw visualization data");
  torch::Tensor vis = ((outputDepth / visMaxDepth_).clamp(0.0, 1.0f) * 255.0f).to(torch::kU8);
  cv::Mat visMat = tensorToCvMatByte(vis.detach().cpu());
  cv::Mat visMatColored;
  cv::applyColorMap(visMat, visMatColored, cv::COLORMAP_INFERNO);

  VisualizationData visData;
  visData.frame = frame;
  visData.depthImage = visMatColored;
  visualisationsQueue_.PushNonBlockingDroppingIfFull(visData, 1);
  t4.stop();

}

void Stereo2DepthProcessor::processing() {
  while(!shutdown_) {
    StereoMeasurement frame;
    if(blocking_){
      while(cameraMeasurementsQueue_.PopBlocking(&frame)) {
        processStereoNetwork(frame);
      }
    }
    else{
      while(cameraMeasurementsQueue_.PopNonBlocking(&frame)) {
        processStereoNetwork(frame);
      }
    }
  }
}

}
