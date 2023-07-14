#ifndef STEREO_DEPTH_STEREO2DEPTH_PROCESSOR_HPP
#define STEREO_DEPTH_STEREO2DEPTH_PROCESSOR_HPP

#include <map>
#include <thread>
#include <atomic>
#include <iostream>

#include <torch/torch.h>
#include <torch/script.h>

#include <okvis/Measurements.hpp>
#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <okvis/QueuedTrajectory.hpp>
#include <okvis/ThreadedSlam.hpp>
#include <okvis/ViSensorBase.hpp>

namespace srl {

class Stereo2DepthProcessor {

struct StereoCameraData {
  cv::Mat leftImage;
  cv::Mat rightImage;
};
typedef okvis::Measurement<StereoCameraData> StereoMeasurement;
struct VisualizationData {
    StereoMeasurement frame;
    cv::Mat depthImage;
};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  Stereo2DepthProcessor(okvis::ViParameters &parameters,
                        std::string modelDir,
                        double visMaxDepth = 10.0);
  virtual ~Stereo2DepthProcessor();

  /**
   * \brief Set the blocking variable that indicates whether the addMeasurement() functions
   *        should return immediately (blocking=false), or only when the processing is complete.
   */
  virtual void setBlocking(bool blocking);


  /// @brief Display some visualisation.
  virtual void display(cv::Mat& image, cv::Mat& stereoDepth);

  /// \name Add measurements to the algorithm.
  /**
   * \brief              Add a set of new image.
   * \param stamp        The image timestamp.
   * \param images       The images.
   * \return             Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) final;

  /// @brief Set the images callback
  /// @param imagesCallback The images callback to register.
  void setDepthImageCallback(const okvis::ViSensorBase::DepthImageCallback & depthImageCallback) {
    depthCallback_ = depthImageCallback;
  }

private:

  /// @brief Processing loops and according threads.
  void processing();
  
  /// @brief Function where the actual neural network predicts the depth from the stereo images
  /// @param frame The stereo images measurement which will then be processed by the neural network
  void processStereoNetwork(StereoMeasurement& frame);

  std::atomic_bool shutdown_{}; ///< True if shutdown requested.
  std::thread processingThread_;
  torch::jit::script::Module depthModel_;
  okvis::ViSensorBase::DepthImageCallback depthCallback_;

  okvis::threadsafe::Queue<StereoMeasurement> cameraMeasurementsQueue_;
  okvis::threadsafe::Queue<VisualizationData> visualisationsQueue_;

  double focalLength_;
  double baseline_;
  double visMaxDepth_;

  std::atomic_bool blocking_;
};

} // namespace srl

#endif //STEREO_DEPTH_STEREO2DEPTH_PROCESSOR_HPP