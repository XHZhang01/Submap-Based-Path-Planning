#ifndef STEREO_DEPTH_PROCESSOR_HPP
#define STEREO_DEPTH_PROCESSOR_HPP

#include <string>

#include <okvis/Parameters.hpp>
#include <okvis/SubmappingInterface.hpp>
#include <okvis/ThreadedSlam.hpp>
#include <okvis/ViInterface.hpp>

#include <okvis/Stereo2DepthProcessor.hpp>

namespace srl {

class Processor: public ::okvis::ViInterface {

public:

  /**
   * \brief Constructor.
   * \param parameters Parameters and settings.
   * @param dBowDir The directory to the DBoW vocabulary.
   */
  Processor(okvis::ViParameters& parameters, 
            std::string dBowDir,
            const se::MapConfig &mapConfig,
            const se::OccupancyDataConfig &dataConfig,
            const se::SubMapConfig &submapConfig,
            std::pair<Eigen::Matrix4d, se::LeicaLidar>* lidarConfig = nullptr,
            std::pair<Eigen::Matrix4d, se::PinholeCamera>* cameraConfig = nullptr);

  ~Processor() = default;

  /// @brief Display some visualisation.
  void display2(cv::Mat& matches, cv::Mat& overhead, cv::Mat& image, cv::Mat& stereoDepth);
  virtual void display(cv::Mat & images, cv::Mat & topDebugImg) final {
    slam_.display(images, topDebugImg);
  }

  /// @brief Print information about the current processing.
  void collectInfo();

  /// @brief Finish processing.
  void finish();

  /// \brief Indicats whether the add functions block. This only supports blocking.
  virtual void setBlocking(bool blocking) final;

  void setT_BS(const okvis::kinematics::Transformation& T_BS);

  /// \brief Runs main processing iteration, call in your main loop.
  bool processFrame();

  /// \name Add measurements to the algorithm.
  /**
   * \brief              Add a set of new image.
   * \param stamp        The image timestamp.
   * \param images       The images.
   * \return             Returns true normally. False, if the previous one has not been processed yet.
   */
  virtual bool addImages(const okvis::Time & stamp, const std::vector<cv::Mat> & images) final;

  /**
   * \brief          Add an IMU measurement.
   * \param stamp    The measurement timestamp.
   * \param alpha    The acceleration measured at this time.
   * \param omega    The angular velocity measured at this time.
   * \return Returns true normally. False if the previous one has not been processed yet.
   */
  virtual bool addImuMeasurement(const okvis::Time & stamp,
                                 const Eigen::Vector3d & alpha,
                                 const Eigen::Vector3d & omega) final;

  /// @brief Set function that handles submaps visualization (blocks version).
  void setSubmapCallback(const okvis::submapCallback &callback);

  /// @brief Set function that handles field slice visualization (blocks version).
  void setFieldSliceCallback(const okvis::fieldCallback &callback);

  /// @brief Set optimized graph callback.
  void setOptimizedGraphCallback(const okvis::ViInterface::OptimisedGraphCallback &callback);

private:
  // TODO: internal graph callback with se update and external callback
  void internalOptimizedGraphCallback(
    const okvis::State &state, 
    const okvis::TrackingState &trackingState,
    std::shared_ptr<const okvis::AlignedMap<okvis::StateId, okvis::State>> alignedMapPtr,
    std::shared_ptr<const okvis::MapPointVector> mapPointVectorPtr);

public:
  okvis::SubmappingInterface se_interface_;
  okvis::ThreadedSlam slam_;
private:
  Stereo2DepthProcessor depthProcessor_;

  okvis::ViInterface::OptimisedGraphCallback optimizedGraphCallback_;
};

}

#endif //STEREO_DEPTH_PROCESSOR_HPP
