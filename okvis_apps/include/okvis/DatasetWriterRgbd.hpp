#ifndef INCLUDE_OKVIS_DATASETWRITERRGBD_HPP_
#define INCLUDE_OKVIS_DATASETWRITERRGBD_HPP_

#include <atomic>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include <boost/filesystem.hpp>

#include <okvis/Frontend.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/cameras/NCameraSystem.hpp>

#include <okvis/RealsenseParameters.hpp>
#include <okvis/RealsenseRgbd.hpp>
#include <okvis/ViInterface.hpp>
#include <okvis/WrappedQueue.hpp>
#include <okvis/timing/Timer.hpp>

namespace okvis {

class DatasetWriterRgbd : public ViInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief      Delete default constructor
   */
  DatasetWriterRgbd() = delete;

  /**
   * @brief      Constructor.
   *
   * @param      parameters  The VI-sensor parameters
   * @param      path        Path to dataset folder.
   */
  DatasetWriterRgbd(okvis::ViParameters &parameters, const std::string &path = "",
                    const RealsenseParameters &realsensePrameters = RealsenseParameters());

  /**
   * @brief      Destroys the object.
   */
  virtual ~DatasetWriterRgbd();

  /**
   * @brief      Adds a depth image.
   *
   * @param[in]  stamp  The timestamp
   * @param[in]  depth  The depth image
   *
   * @return     True when current depth image has been processed normally
   */
  virtual bool addDepthImage(const okvis::Time &stamp, const cv::Mat &depth) final;

  /**
   * @brief      Adds an rgb image.
   *
   * @param[in]  stamp  The timestamp
   * @param[in]  rgb    The rgb
   *
   * @return     True when the rgb image has been processed normally
   */
  virtual bool addRGBImage(const okvis::Time &stamp, const cv::Mat &rgb) final;

  /**
   * @brief      Adds images.
   *
   * @param[in]  stamp   The image time stamp
   * @param[in]  images  The images
   *
   * @return     True when current set of images has been processed normally.
   */
  virtual bool addImages(const okvis::Time &stamp, const std::vector<cv::Mat> &images) override final;

  /**
   * @brief      Adds an imu measurement.
   *
   * @param[in]  stamp  The timestamp
   * @param[in]  accel  The accelerometer measurement
   * @param[in]  gyro   The gyro measurement
   *
   * @return     True when current set of images has been processed normally.
   */
  virtual bool addImuMeasurement(const okvis::Time &stamp, const Eigen::Vector3d &accel,
                                 const Eigen::Vector3d &gyro) override final;

  /*
  virtual void setBlocking(bool blocking) override final;
  */

  /**
   * @brief      Colourises an input depth frame using the method described in
   * [https://dev.intelrealsense.com/docs/depth-image-compression-by-colorization-for-intel-realsense-depth-cameras#section-3-2-depth-image-recovery-from-colorized-depth-images-in-c]
   *
   * @param[in]  inputDepth       The input depth
   * @param[out] colourisedDepth  The colourised depth
   */
  static void colouriseDepth(const cv::Mat &inputDepth, cv::Mat &colourisedDepth);

  /**
   * @brief      Displays the incoming images
   */
  virtual void display() override final;

private:
  /// @brief Main processing loop.
  void processing();
  okvis::ViParameters parameters_;                                            ///< All VI parameters
  const RealsenseParameters realsensePrameters_;                              ///< Realsense Parameters
  std::atomic_bool shutdown_;                                                 ///< True if shutdown requested.
  std::thread processingThread_;                                              ///< Thread running the processing loop.
  MyQueue<std::vector<okvis::CameraMeasurement>> cameraMeasurementsReceived_; ///< Camera frames
  MyQueue<okvis::DepthCameraMeasurement> depthMeasurementsReceived_;          ///< Depth frames
  MyQueue<okvis::CameraMeasurement> rgbMeasurementsReceived_;                 ///< RGB frames
  MyQueue<std::vector<okvis::CameraMeasurement>> visualisations_;             ///< Queue for visualition
  MyQueue<okvis::DepthCameraMeasurement> depthVisualitations_; ///< Queue for visualising the Recorded depth frames
  MyQueue<okvis::ImuMeasurement> imuMeasurementsReceived_;     ///< IMU measurements
  std::stringstream datasetDirectory_;                         ///< Path to the dataset.
  std::stringstream imuDirectory_;                             ///< Directory to the IMU data file.
  std::string depthDirectory_;                                 ///< Directory to the Depth data
  std::string rgbDirectory_;                                   ///< Directory to save the RGB frames
  std::ofstream imuCsv_;                                       ///< IMU data file.
  std::ofstream depthCsv_;                                     ///< Depth data file.
  std::ofstream rgbCsv_;                                       ///< RGB data file.
  std::vector<std::string> camDirectories_;                    ///< Directories to the camera data.
  std::vector<std::ofstream> camCsvs_;                         ///< Camera data files.
};

} // namespace okvis

#endif
