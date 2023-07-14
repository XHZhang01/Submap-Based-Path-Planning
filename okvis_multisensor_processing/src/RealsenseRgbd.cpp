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
 * @file RealsenseRgbd.cpp
 * @brief Source file for the Realsense Rgbd class.
 * @author Simon Schaefer
 */

#include <thread>
#include <okvis/RealsenseRgbd.hpp>

namespace okvis {

RealsenseRgbd::RealsenseRgbd(okvis::Realsense::SensorType sensorType, int rate) 
    : Realsense(sensorType, rate, rate) { 
    
  irRate_ = 2*rate; // adjust rates so alternation works
  rgbRate_ = 2*rate; // adjust rates so alternation works
  
  OKVIS_ASSERT_TRUE(Exception, sensorType == okvis::Realsense::SensorType::D455, 
      "Depth channel not supported")

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

void RealsenseRgbd::processFrame(const rs2::frame &frame) {
  if(!streaming_) {
    return; // be sure the emitter stuff is set up first
  }
  if(!checkFrameAndUpdate(frame)) {
    return;
  }

  if(const auto &fs = frame.as<rs2::frameset>()) {
    // Fetch the infrared camera data.
    const auto ir0 = fs.get_infrared_frame(1);
    const auto ir1 = fs.get_infrared_frame(2);

    // Status of the emitter
    const bool supports_md_0 = ir0.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
    const bool supports_md_1 = ir1.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
    if(supportsMetadata_ && (!supports_md_0 || !supports_md_1)) {
      LOG(WARNING)<< "reading the emitter mode is not supported, turning off emitter";
      rs2::device selected_device = profile_.get_device();
      auto depth_sensor = selected_device.first<rs2::depth_sensor>();
      depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 0.f);
      depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
      depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f);
      supportsMetadata_ = false;
      return;
    }

    bool emitter_status_0 = false;
    bool emitter_status_1 = false;
    if(supportsMetadata_) {
      emitter_status_0 = ir0.get_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
      emitter_status_1 = ir1.get_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
    }

    // Initialise the frame count for the rate divider. Stefan does not like static vars.
    if(frameCountStart_ == 0) {
      frameCountStart_ = fs.get_frame_number();
    }

    // Process IR or RGB frames, depending on the emitter status and frame number.
    // todo:  RS2_FRAME_METADATA_SENSOR_TIMESTAMP does not work for global time???
    int irFrame_modulo = (ir0.get_frame_number() - frameCountStart_) % irStride_; //Here is when we downsample the infrarred! From 60Hz to irRate
    int depthFrame_modulo = (ir0.get_frame_number() - frameCountStart_) % rgbStride_; //Here is when we downsample the infrarred! From 60Hz to depthRate
    if ((!supportsMetadata_ || (!emitter_status_0 && !emitter_status_1)) && (irFrame_modulo == 0) && imuStreamStarted_) {
      processIR_(fs);
    } else if ((!supportsMetadata_ || (emitter_status_0 && emitter_status_1)) && (depthFrame_modulo == 1) && imuStreamStarted_) {
      processRgb_(fs);
      processDepth_(fs);
    }
  }

  // Otherwise the frame is a motion frame. Collect and process the IMU data.
  bool imu_valid = processImu_(frame);
  if(!imuStreamStarted_ && imu_valid) {
    imuStreamStarted_ = true;
  }
}

bool RealsenseRgbd::processDepth_(const rs2::frameset &frame) {
  const auto depth_frame = frame.get_depth_frame();  // implicitly assumes that fs is not None
  if (!depth_frame)
    return false;
  okvis::Time ts;
  computeTimeStampFromFrame_(frame, RS2_FRAME_METADATA_SENSOR_TIMESTAMP, ts);

  // todo: RS2_FRAME_METADATA_SENSOR_TIMESTAMP or
  auto depth_mat = frame2Mat(depth_frame, 640, 480, CV_16UC1).clone();
  depth_mat.convertTo(depth_mat, CV_32F);
  depth_mat = depth_mat * depth_scale_ * 1000;  // now in mm

  depthCallback_(ts, depth_mat);

  // Also save the depth frame aligned to the RGB
  // rs2::align align_to_color(RS2_STREAM_COLOR);
  // const auto aligned_frameset = align_to_color.process(frame);
  // const auto depth_frame_aligned = aligned_frameset.get_depth_frame();

  // Convert to OpenCV Mat
  // auto depth_frame_aligned_mat = frame2Mat(depth_frame_aligned, 640, 480, CV_16UC1).clone();
  // depth_frame_aligned_mat.convertTo(depth_frame_aligned_mat, CV_32F);
  // depth_frame_aligned_mat = depth_frame_aligned_mat * depth_scale_;
  // depthCallback_(ts, depth_frame_aligned_mat);
  return true;
}

bool RealsenseRgbd::startStreaming() {
  bool success = startStreaming_(cv::Size(640, 480), irRate_, cv::Size(1280, 720), rgbRate_);
  cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY, irRate_);
  profile_ = pipe_.start(cfg_, std::bind(&Realsense::processFrame, this, std::placeholders::_1));

  rs2::device selected_device = profile_.get_device();
  auto depth_sensor = selected_device.first<rs2::depth_sensor>();
  // Set maximum laser power [0, 360].
  depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 0.f);
  depth_sensor.set_option(RS2_OPTION_LASER_POWER, 360.f);
  // Enable the emitter
  depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
  // Enable alternating emitter mode
  depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 1.f);
  // Enable global timestamps, shared across the sensors.
  selected_device.first<rs2::depth_sensor>().set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);
  selected_device.first<rs2::motion_sensor>().set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);
  selected_device.first<rs2::color_sensor>().set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);

  // Make sure depth scale is initialised properly.
  depth_scale_ = selected_device.first<rs2::depth_sensor>().get_depth_scale();

  streaming_ = true;
  return success;
}
}
