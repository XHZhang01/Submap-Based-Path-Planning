#include <okvis/DatasetWriterRgbd.hpp>
namespace okvis {

DatasetWriterRgbd::DatasetWriterRgbd(ViParameters &parameters, const std::string &path,
                                     const RealsenseParameters &realsensePrameters)
    : parameters_(parameters), realsensePrameters_(realsensePrameters) {
  // check valid path
  OKVIS_ASSERT_TRUE(Exception, boost::filesystem::is_directory(path),
                    "provided path: " << path << " not a valid directory")

  // create dataset folder with timestamp
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  datasetDirectory_ << path << "/" << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  boost::filesystem::create_directory(datasetDirectory_.str());

  // create imu csv file
  boost::filesystem::create_directory(datasetDirectory_.str() + "/imu0/");
  imuDirectory_ << datasetDirectory_.str() << "/imu0/";
  imuCsv_.open(imuDirectory_.str() + "data.csv");
  OKVIS_ASSERT_TRUE(Exception, imuCsv_.good(), "couldn't open " << imuDirectory_.str() << "data.csv")
  imuCsv_ << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z "
             "[rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
          << std::endl;

  // create image folders, CSVs and image queues
  for (size_t i = 0; i < parameters.nCameraSystem.numCameras(); ++i) {
    std::stringstream camDirectory;
    camDirectory << datasetDirectory_.str() << "/cam" << i << "/";
    camDirectories_.push_back(camDirectory.str());
    boost::filesystem::create_directory(camDirectory.str());
    boost::filesystem::create_directory(camDirectory.str() + "data/");

    camCsvs_.push_back(std::ofstream(camDirectory.str() + "data.csv"));
    OKVIS_ASSERT_TRUE(Exception, camCsvs_.back().good(), "couldn't open " << camDirectory.str() << "data.csv")
    camCsvs_.back() << "#timestamp [ns],filename" << std::endl;
  }

  // ToDo -> Agree on VI-O parameters
  // Path for the depth data
  depthDirectory_ = datasetDirectory_.str() + "/depth0/";
  boost::filesystem::create_directory(depthDirectory_);
  boost::filesystem::create_directory(depthDirectory_ + "data/");
  depthCsv_.open(depthDirectory_ + "data.csv");
  OKVIS_ASSERT_TRUE(Exception, depthCsv_.good(), "couldn't open " << depthDirectory_ << "data.csv")

  // Create the RGB folders
  if (realsensePrameters_.enable_rgb) {
    rgbDirectory_ = datasetDirectory_.str() + "/rgb0/";
    boost::filesystem::create_directory(rgbDirectory_);
    boost::filesystem::create_directory(rgbDirectory_ + "data/");
    rgbCsv_.open(rgbDirectory_ + "data.csv");
    OKVIS_ASSERT_TRUE(Exception, depthCsv_.good(), "couldn't open " << rgbDirectory_ << "data.csv")
  }

  // start processing thread
  shutdown_ = false;
  processingThread_ = std::thread(&DatasetWriterRgbd::processing, this);
}

DatasetWriterRgbd::~DatasetWriterRgbd() {
  // shutdown queues
  imuMeasurementsReceived_.Shutdown();
  cameraMeasurementsReceived_.Shutdown();
  depthMeasurementsReceived_.Shutdown();

  if (realsensePrameters_.enable_rgb) {
    rgbMeasurementsReceived_.Shutdown();
  }
  visualisations_.Shutdown();
  depthVisualitations_.Shutdown();

  // finish writing what's already in the queues
  shutdown_ = true;
  processingThread_.join();

  // close CSV files
  imuCsv_.close();
  depthCsv_.close();
  if (realsensePrameters_.enable_rgb) {
    rgbCsv_.close();
  }
  for (size_t i = 0; i < camCsvs_.size(); ++i) {
    camCsvs_.at(i).close();
  }
}

bool DatasetWriterRgbd::addDepthImage(const okvis::Time &stamp, const cv::Mat &depth) {
  // ToDo -> Consider having constructors
  DepthCameraMeasurement depth_measurement;

  // Placeholder for RGB image ?
  /*
  depth_measurement.measurement.image = rgbImage;
  */
  depth_measurement.measurement.depthImage = depth;
  depth_measurement.measurement.deliversKeypoints = false;
  depth_measurement.timeStamp = stamp;
  depth_measurement.sensorId = 0;

  const int depthQueueSize = 100;
  if (depthMeasurementsReceived_.PushNonBlockingDroppingIfFull(depth_measurement, size_t(depthQueueSize))) {
    LOG(WARNING) << "Depth measurement drop";
    return false;
  }

  return true;
}

bool DatasetWriterRgbd::addRGBImage(const okvis::Time &stamp, const cv::Mat &rgb) {
  if (!realsensePrameters_.enable_rgb)
    return false;

  // ToDo -> Consider having constructors
  CameraMeasurement rgb_measurement;

  rgb_measurement.measurement.image = rgb;
  rgb_measurement.timeStamp = stamp;
  rgb_measurement.sensorId = 0;
  rgb_measurement.measurement.deliversKeypoints = false;

  const int rgbQueueSize = 100;
  if (rgbMeasurementsReceived_.PushNonBlockingDroppingIfFull(rgb_measurement, size_t(rgbQueueSize))) {
    LOG(WARNING) << "RGB measurement drop";
    return false;
  }

  return true;
}

void DatasetWriterRgbd::colouriseDepth(const cv::Mat &inputDepth, cv::Mat &colourisedDepth) {
  const double depthMin = 0.0;
  const double depthMax = 5.0;

  colourisedDepth = cv::Mat(inputDepth.rows, inputDepth.cols, CV_8UC3);

  colourisedDepth.forEach<cv::Point3_<uint8_t>>([&](cv::Point3_<uint8_t> &p, const int *position) -> void {
    // Convert Depth from TUM format to meters.
    const double d = inputDepth.at<uint16_t>(position[0], position[1]) / 5000.0;
    const double d_normal = (d - depthMin) * 1529 / (depthMax - depthMin);

    if (d == 0) {
      p.x = 0;
      p.y = 0;
      p.z = 0;
      return;
    }

    // Red channel
    if (((0 <= d_normal) && (d_normal <= 255)) || ((1275 < d_normal) && (d_normal <= 1529))) {
      p.x = 255;
    } else if ((255 < d_normal) && (d_normal <= 510)) {
      p.x = 255 - d_normal;
    } else if ((510 <= d_normal) && (d_normal <= 1020)) {
      p.x = 0;
    } else {
      p.x = d_normal - 1020;
    }

    // Green channel
    if ((0 <= d_normal) && (d_normal <= 255)) {
      p.y = d_normal;
    } else if ((255 < d_normal) && (d_normal <= 510)) {
      p.y = 255;
    } else if ((510 <= d_normal) && (d_normal <= 765)) {
      p.y = 765 - d_normal;
    } else {
      p.y = 0;
    }

    // Blue channel
    if ((0 <= d_normal) && (d_normal <= 765)) {
      p.z = 0;
    } else if ((765 < d_normal) && (d_normal <= 1020)) {
      p.z = d_normal - 765;
    } else if ((1020 <= d_normal) && (d_normal <= 1275)) {
      p.z = 255;
    } else {
      p.z = 1275 - d_normal;
    }
  });
}

void DatasetWriterRgbd::display() {
  std::vector<CameraMeasurement> frames;
  okvis::DepthCameraMeasurement depthFrame;

  // Visualise the stereo frames
  if (visualisations_.PopNonBlocking(&frames) && depthVisualitations_.PopNonBlocking(&depthFrame)) {
    for (size_t i = 0; i < frames.size(); ++i) {
      std::stringstream windowname;
      windowname << "cam" << i;
      cv::imshow(windowname.str(), frames.at(i).measurement.image);
    }

    cv::Mat colourisedDepth;
    colouriseDepth(depthFrame.measurement.depthImage, colourisedDepth);
    cv::imshow("Depth frame", colourisedDepth);
  }
}

bool DatasetWriterRgbd::addImuMeasurement(const Time &stamp, const Eigen::Vector3d &alpha,
                                          const Eigen::Vector3d &omega) {
  okvis::ImuMeasurement imu_measurement;
  imu_measurement.measurement.accelerometers = alpha;
  imu_measurement.measurement.gyroscopes = omega;
  imu_measurement.timeStamp = stamp;

  const int imuQueueSize = 100;
  if (imuMeasurementsReceived_.PushNonBlockingDroppingIfFull(imu_measurement, size_t(imuQueueSize))) {
    LOG(WARNING) << "imu measurement drop ";
    return false;
  }
  return true;
}

bool DatasetWriterRgbd::addImages(const Time &stamp, const std::vector<cv::Mat> &images) {
  // assemble frame
  const size_t numCameras = images.size();
  std::vector<okvis::CameraMeasurement> frames(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    frames.at(i).measurement.image = images.at(i);
    frames.at(i).timeStamp = stamp;
    frames.at(i).sensorId = int(i);
    frames.at(i).measurement.deliversKeypoints = false;
  }

  const int cameraInputQueueSize = 100;
  if (cameraMeasurementsReceived_.PushNonBlockingDroppingIfFull(frames, cameraInputQueueSize)) {
    LOG(WARNING) << "frame drop";
    return false;
  }
  return true;
}

void DatasetWriterRgbd::processing() {
  while (!shutdown_) {

    // Save the IMU measurements
    ImuMeasurement imuMeasurement;
    while (imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement)) {
      imuCsv_ << imuMeasurement.timeStamp.toNSec() << "," << imuMeasurement.measurement.gyroscopes[0] << ","
              << imuMeasurement.measurement.gyroscopes[1] << "," << imuMeasurement.measurement.gyroscopes[2] << ","
              << imuMeasurement.measurement.accelerometers[0] << "," << imuMeasurement.measurement.accelerometers[1]
              << "," << imuMeasurement.measurement.accelerometers[2] << std::endl;
    }

    // Save the Camera frames
    std::vector<CameraMeasurement> frames;
    while (cameraMeasurementsReceived_.PopNonBlocking(&frames)) {
      visualisations_.PushNonBlockingDroppingIfFull(frames, 1);
      const size_t numCameras = frames.size();
      for (size_t i = 0; i < numCameras; ++i) {
        // write text file data
        uint64_t timestamp = frames.at(i).timeStamp.toNSec();
        camCsvs_.at(i) << timestamp << "," << timestamp << ".png" << std::endl;

        // write image png
        std::stringstream imagename;
        imagename << camDirectories_.at(i) << "data/" << timestamp << ".png";
        cv::imwrite(imagename.str(), frames.at(i).measurement.image);
      }
    }

    // Save the Depth frames
    DepthCameraMeasurement depthMeasurement;
    while (depthMeasurementsReceived_.PopNonBlocking(&depthMeasurement)) {
      depthVisualitations_.PushNonBlockingDroppingIfFull(depthMeasurement, 1);

      // Get the timestamp
      const uint64_t timestamp = depthMeasurement.timeStamp.toNSec();

      // Write CSV data
      depthCsv_ << timestamp << "," << timestamp << ".png" << std::endl;

      // Save the .png to disk

      std::stringstream depth_image_name;
      depth_image_name << depthDirectory_ << "data/" << timestamp << ".png";
      cv::imwrite(depth_image_name.str(), depthMeasurement.measurement.depthImage);
    }

    // Save the RGB frames
    if (realsensePrameters_.enable_rgb) {
      CameraMeasurement rgbMeasurement;
      while (rgbMeasurementsReceived_.PopNonBlocking(&rgbMeasurement)) {
        // Get the timestamp
        const uint64_t timestamp = rgbMeasurement.timeStamp.toNSec();

        // Write CSV data
        rgbCsv_ << timestamp << "," << timestamp << ".png" << std::endl;

        // Save the .png to disk
        std::stringstream rgb_image_name;
        rgb_image_name << rgbDirectory_ << "data/" << timestamp << ".png";
        cv::imwrite(rgb_image_name.str(), rgbMeasurement.measurement.image);
      }
    }
  }
}

} // namespace okvis