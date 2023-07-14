/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London *
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
 *  Created on: April 10, 2020
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file Realsense.cpp
 * @brief Source file for the Realsense class.
 * @author Stefan Leutenegger
 */

#include <thread>
#include <okvis/Realsense.hpp>


namespace okvis {

Realsense::Realsense(SensorType sensorType, int irRate, int rgbRate): 
    streaming_(false), started_(false), sensorType_(sensorType),
    irRate_(irRate), rgbRate_(rgbRate) {

  if(irRate_ != 60 && irRate_ != 30 && irRate_ != 15 && irRate_ != 5) {
    if(irRate_ == 10) {
      irRate_ = 30;
      irStride_ = 3;
    } else if (irRate_ == 20) {
      irRate_ = 60;
      irStride_ = 3;
    } else {
      LOG(WARNING) << "IR rate " << irRate_ << "not supported, setting to default 15 Hz";
      irRate_ = 15;
    }
  }
  if(rgbRate_ != 60 && rgbRate_ != 30 && rgbRate_ != 15 && rgbRate_ != 5) {
    if(rgbRate_ == 10) {
      rgbRate_ = 30;
      rgbStride_ = 3;
    } else if (irRate_ == 20) {
      rgbRate_ = 60;
      rgbStride_ = 3;
    } else {
      LOG(WARNING) << "RGB rate " << rgbRate_ << "not supported, setting to default 15 Hz";
      rgbRate_ = 15;
    }
  }
}

Realsense::~Realsense() {
  if(streaming_) {
    stopStreaming();
  }
}

void Realsense::setSensorType(Realsense::SensorType sensorType) {
  sensorType_ = sensorType;
}

void Realsense::processFrame(const rs2::frame& frame) {
  if(!streaming_) {
    return; // be sure the emitter stuff is set up first
  }
  if(!checkFrameAndUpdate(frame)) {
    return;
  }
  if (const auto &fs = frame.as<rs2::frameset>()) {
    if(frameCountStart_ == 0) {
      frameCountStart_ = fs.get_frame_number();
    }
    processIR_(fs);
    if(sensorType_ == okvis::Realsense::SensorType::D455) {
      processRgb_(fs);
    }
  }
  processImu_(frame);
}

bool Realsense::checkFrameAndUpdate(const rs2::frame &frame) {
  // check frame metadata available
  if(hasDeviceTimestamps_ && !frame.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)) {
    LOG(WARNING) << "Device timestamps not available. Switching to host timestamps";
    hasDeviceTimestamps_ = false;
    return false;
  }

  // get timestamp(s)
  const double host_ts_ms  = frame.get_timestamp();
  int64_t sensor_ts_us = hasDeviceTimestamps_ ?
      frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP) : uint64_t(host_ts_ms*1000.0);

  // delay start until first image
  if(firstTimeUs_==0) {
    std::cout << "waiting for first image ... " << std::endl;
    firstTimeUs_ = sensor_ts_us;
  }
  if(!started_) {
    if (const auto &fs = frame.as<rs2::frameset>()) {
      const auto ir0 = fs.get_infrared_frame(1);
      const auto ir1 = fs.get_infrared_frame(2);
      std::vector<cv::Mat> images(2);
      if (ir0 && ir1) {
        started_ = true;
        std::cout << "first image received after " << (sensor_ts_us-firstTimeUs_)*1.0e-6 << " seconds " << std::endl;
      }
    }
  }

  // estimate host time offset
  uint64_t measuredHostTimeOffsetUs = (uint64_t(host_ts_ms*1000.0)-sensor_ts_us);
  hostTimeOffsetUs_ = (frameCounter_ * hostTimeOffsetUs_ + measuredHostTimeOffsetUs)/(frameCounter_+1);
  frameCounter_ = std::min(frameCounter_ + 1, 1000); // saturate at N=1000 -- this should probably be configurable.

  return true;
}

uint64_t Realsense::computeTimeStampFromFrame_(const rs2::frame &frame, const rs2_frame_metadata_value frameMetadata, okvis::Time& ts) {
  const double host_ts_ms  = frame.get_timestamp();
  uint64_t sensor_ts_us = hasDeviceTimestamps_ ?
      frame.get_frame_metadata(frameMetadata) : uint64_t(host_ts_ms*1000.0);
  if (hasDeviceTimestamps_) {
    ts.fromNSec(sensor_ts_us * 1000 + hostTimeOffsetUs_ * 1000);
  } else {
    ts.fromNSec(uint64_t(host_ts_ms * 1000.0) * 1000);
  }
  return sensor_ts_us;
}


bool Realsense::processIR_(const rs2::frameset &frame) {
  // Note that the frame rate of different sensors might be different, therefore
  // check for the correctness of frame index order and skip frames if required.
  const auto ir0 = frame.get_infrared_frame(1);  // implicitly assuming fs is not None
  const auto ir1 = frame.get_infrared_frame(2);
  if (!ir0 || !ir1)
    return false;
    
  int irFrame_modulo = (ir0.get_frame_number() - frameCountStart_) % irStride_; //Here is when we downsample the rgb!
  if(irFrame_modulo != 0) {
    return false;
  }

  okvis::Time ts;
  const uint64_t sensor_ts_us = computeTimeStampFromFrame_(frame, RS2_FRAME_METADATA_SENSOR_TIMESTAMP, ts);
  int ir0_image_index = ir0.get_frame_number();
  int ir1_image_index = ir1.get_frame_number();
  if (ir0_image_index == ir1_image_index && lastImgIdx_ < ir0_image_index) {
    lastImgIdx_ = ir0_image_index;
    if (!started_) {
      started_ = true;
      std::cout << "first image received after " << (sensor_ts_us - firstTimeUs_) * 1.0e-6 << " seconds"
                << std::endl;
    }
    std::vector<cv::Mat> images(2);
    images.at(0) = frame2Mat(ir0, 640, 480, CV_8UC1).clone();
    images.at(1) = frame2Mat(ir1, 640, 480, CV_8UC1).clone();
    imagesCallback_(ts, images);
  }
  return true;
}

bool Realsense::processRgb_(const rs2::frameset &frame) {
  const auto rgb = frame.get_color_frame();  // implicitly assuming fs is not None
  if (!rgb)
    return false;
    
  int rgbFrame_modulo = (rgb.get_frame_number() - frameCountStart_) % rgbStride_; //Here is when we downsample the rgb!
  if(rgbFrame_modulo != 0) {
    return false;
  }

  // todo: should it be RS2_FRAME_METADATA_SENSOR_TIMESTAMP here?
  okvis::Time ts;
  computeTimeStampFromFrame_(frame, RS2_FRAME_METADATA_SENSOR_TIMESTAMP, ts);
  int rgb_image_index = rgb.get_frame_number();
  if (lastRGBImgIdx_ < rgb_image_index) {
    lastRGBImgIdx_ = rgb_image_index;
    cv::Mat rgb_image = frame2Mat(rgb, 1280, 720, CV_8UC3);
    rgbCallback_(ts, rgb_image);
  }
  return true;
}

bool Realsense::processImu_(const rs2::frame &frame) {
  auto motion = frame.as<rs2::motion_frame>();
  okvis::Time ts;
  bool success = false;
  const uint64_t sensor_ts_us = computeTimeStampFromFrame_(frame, RS2_FRAME_METADATA_FRAME_TIMESTAMP, ts);

  // If casting succeeded and the arrived frame is from gyro stream
  if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO
      && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {

    int gyrIdx = frame.get_frame_number();
    if(lastGyrIdx_ >= gyrIdx) LOG(WARNING) << "gyro sequence number out of order";
    lastGyrIdx_ = gyrIdx;

    // Get gyro measures
    rs2_vector gyro_data = motion.get_motion_data();
    //std::cout << ts << " gyro = " << gyro_data << std::endl;
    okvis::Measurement<Eigen::Matrix<double,3,1,Eigen::DontAlign>> gyr{ts,
    Eigen::Vector3d(gyro_data.x, gyro_data.y, gyro_data.z)};

    // buffer it
    gyrBuffer_[sensor_ts_us] = gyr;
  }
  // If casting succeeded and the arrived frame is from accelerometer stream
  if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL
      && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {

    int accIdx = frame.get_frame_number();
    if(lastAccIdx_ >= accIdx) LOG(WARNING) << "accelerometer sequence number out of order";
    lastAccIdx_ = accIdx;

    // Get accelerometer measures
    rs2_vector accel_data = motion.get_motion_data();
    //std::cout << ts << " accel = " << accel_data << std::endl;
    okvis::Measurement<Eigen::Matrix<double,3,1,Eigen::DontAlign>> acc{ts,
    Eigen::Vector3d(accel_data.x, accel_data.y, accel_data.z)};

    // buffer it
    accBuffer_[sensor_ts_us] = acc;

    // try aligning interpolated acceleration measurement. Slightly counter-intuitive to do it
    // while processing the acceleration measurements, but we have to wait for them
    // to interpolate gyro measurements.
    auto iterAcc = accBuffer_.begin();
    auto iterGyr = gyrBuffer_.begin();
    while(iterAcc != accBuffer_.end() && iterGyr != gyrBuffer_.end()) {
      auto iterAccNext = iterAcc;
      iterAccNext++;
        if(iterAccNext == accBuffer_.end()) {
        break;
      }
      // check if we have old enough acceleration measurments to interpolate
      if(iterGyr->first < iterAcc->first) {
        LOG(WARNING) << "discarding gyro measurement at t=" << iterAcc->second.timeStamp;
        iterGyr = gyrBuffer_.erase(iterGyr);
        continue;
      }
      // check if we have new enough acceleration measurments to interpolate
      if(iterGyr->first > iterAccNext->first) {
        iterAcc = accBuffer_.erase(iterAcc);
        continue;
      }
      // now that things are aligned, we can interpolate
      double r = double(iterGyr->first-iterAcc->first)/double(iterAccNext->first-iterAcc->first);
      Eigen::Vector3d accInterp =
      (1-r)*iterAcc->second.measurement + r*iterAccNext->second.measurement;
      ImuMeasurement imuMeasurement{
          iterGyr->second.timeStamp, ImuSensorReadings{iterGyr->second.measurement, accInterp}};
      iterGyr = gyrBuffer_.erase(iterGyr);

      // call callback
      //std::cout << "imu " << imuMeasurement.timeStamp << std::endl;
      imuCallback_(imuMeasurement.timeStamp, imuMeasurement.measurement.accelerometers,
                   imuMeasurement.measurement.gyroscopes);
      success = true;
    }
  }
  return success;
}

bool Realsense::startStreaming() {
  std::cout << irRate_ << " " << rgbRate_ << std::endl;
  bool success = startStreaming_(cv::Size(640, 480), irRate_, cv::Size(1280, 720), rgbRate_);
  profile_ = pipe_.start(cfg_, std::bind(&Realsense::processFrame, this, std::placeholders::_1));

  // switch off emitter
  rs2::device selected_device = profile_.get_device();
  auto depth_sensor = selected_device.first<rs2::depth_sensor>();
  depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 0.f);
  depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, false);
  depth_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);

  streaming_ = true;
  return success;
}

bool Realsense::stopStreaming() {
  // switch off alternating emitter
  rs2::device selected_device = profile_.get_device();
  auto depth_sensor = selected_device.first<rs2::depth_sensor>();
  depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 0.f);
  // Stop the pipeline
  pipe_.stop();
  streaming_ = false;
  started_ = false;
  return true;
}

bool Realsense::isStreaming() {
  return streaming_;
}

std::string Realsense::getDeviceName(const rs2::device& dev) {
  // Each device provides some information on itself, such as name:
  std::string name = "Unknown Device";
  if (dev.supports(RS2_CAMERA_INFO_NAME))
     name = dev.get_info(RS2_CAMERA_INFO_NAME);

  // and the serial number of the device:
  std::string sn = "########";
  if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
    sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

  return name + " " + sn;
}

cv::Mat Realsense::frame2Mat(const rs2::frame &frame, const int width, const int height, const int format) {
  const cv::Size size(width, height);
  const auto stride = cv::Mat::AUTO_STEP;
  const cv::Mat cvFrame(size, format, (void *) frame.get_data(), stride);
  return cvFrame;
}

bool Realsense::checkSupport() {
  bool found_gyro = false;
  bool found_accel = false;
  bool found_ir = false;

  bool found_rgb = false;
  if (!rgbCallback_) found_rgb = true;   // if no callback, don't look for RGB sensor

  rs2::context ctx;
  for (const auto& dev : ctx.query_devices())
  {
    // The same device should support gyro and accel
    found_gyro = false;
    found_accel = false;
    for (const auto& sensor : dev.query_sensors())
    {
      for (const auto& profile : sensor.get_stream_profiles())
      {
        if (profile.stream_type() == RS2_STREAM_GYRO)
          found_gyro = true;
        if (profile.stream_type() == RS2_STREAM_ACCEL)
          found_accel = true;
        if (profile.stream_type() == RS2_STREAM_INFRARED)
          found_ir = true;
        if (profile.stream_type() == RS2_STREAM_COLOR)
          found_rgb = true;
      }
    }
    if (found_gyro && found_accel && found_ir && found_rgb)
      break;
  }
  return found_gyro && found_accel && found_ir && found_rgb;
}

bool Realsense::startStreaming_(const cv::Size& irSize, const uint irFps, const cv::Size& rgbSize, const uint rgbFps) {
  // get device connected
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  if(devices.size() == 0) {
    std::cerr << "No device connected, please connect a RealSense device" << std::endl;
    return false;
  }
  std::cout << "Found the following devices:" << std::endl;
  int index = 0;
  for (const rs2::device& device : devices) {
    std::cout << "  " << index++ << " : " << getDeviceName(device) << std::endl;
  }

  OKVIS_ASSERT_TRUE(Exception, checkSupport(), "IR stereo or IMU not supported by sensor")
  OKVIS_ASSERT_TRUE(Exception, imagesCallback_, "no add image callback registered")
  OKVIS_ASSERT_TRUE(Exception, imuCallback_, "no add IMU callback registered")

  // Add streams of gyro and accelerometer to configuration
  cfg_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
  cfg_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 400);

  // add IR images
  cfg_.enable_stream(RS2_STREAM_INFRARED, 1, irSize.width, irSize.height, RS2_FORMAT_Y8, irFps);
  cfg_.enable_stream(RS2_STREAM_INFRARED, 2, irSize.width, irSize.height, RS2_FORMAT_Y8, irFps);

  // add color image, if callback is defined
  if(rgbCallback_) {
    cfg_.enable_stream(RS2_STREAM_COLOR, rgbSize.width, rgbSize.height, RS2_FORMAT_BGR8, rgbFps);
  }
  return true;
}

void Realsense::setHasDeviceTimestamps(const bool hasDeviceTimestamps) {
  hasDeviceTimestamps_ = hasDeviceTimestamps;
}

}
