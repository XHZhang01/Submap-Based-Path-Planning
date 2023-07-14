/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
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
 * @file okvis_app_realsense_recorder.cpp
 * @brief This file records a dataset.
 
 Compatible and tested with D435i

 * @author Stefan Leutenegger
 */

#include <iostream>
#include <signal.h>

#include <opencv2/highgui/highgui.hpp>

#include <okvis/Realsense.hpp>
#include <okvis/RealsenseRgbd.hpp>
#include <okvis/DatasetWriter.hpp>
#include <okvis/ViParametersReader.hpp>

static std::atomic_bool shtdown; ///< Shutdown requested?

/// \brief Main.
/// \param argc argc.
/// \param argv argv.
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  // command line arguments
  if (argc != 3 && argc != 4) {
    LOG(ERROR)<<"Usage: ./" << argv[0] << " configuration-yaml-file path/to/dataset [-rgb] [-rgbd]";
    return EXIT_FAILURE;
  }
  bool rgb = false;
  bool depth = false;
  if (argc == 4) {
    if(strcmp(argv[3], "-rgb")==0) {
      rgb = true;
    }
    if(strcmp(argv[3], "-rgbd")==0) {
      rgb = true;
      depth = true;
    }
  }

  try {

    // realsense sensor
    std::unique_ptr<okvis::Realsense> realsense;
    if(!depth) {
      realsense.reset(new okvis::Realsense(okvis::Realsense::SensorType::D455));
    } else {
      realsense.reset(new okvis::RealsenseRgbd(okvis::Realsense::SensorType::D455));
    }
    realsense->setHasDeviceTimestamps(false);

    // construct OKVIS side
    std::string configFilename(argv[1]);
    okvis::ViParametersReader viParametersReader(configFilename);
    okvis::ViParameters parameters;
    viParametersReader.getParameters(parameters);

    // dataset writer
    std::string path(argv[2]);
    okvis::DatasetWriter datasetWriter(parameters, path, rgb, depth);

    // connect sensor to datasetWriter
    realsense->setImuCallback(
          std::bind(&okvis::DatasetWriter::addImuMeasurement, &datasetWriter,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    realsense->setImagesCallback(
          std::bind(&okvis::DatasetWriter::addImages, &datasetWriter, std::placeholders::_1,
                    std::placeholders::_2));
    if(rgb) {
      realsense->setRgbImageCallback(
          std::bind(&okvis::DatasetWriter::addRGBImage, &datasetWriter, std::placeholders::_1,
                    std::placeholders::_2));
    }
    if(depth) {
      realsense->setDepthImageCallback(
          std::bind(&okvis::DatasetWriter::addDepthImage, &datasetWriter, std::placeholders::_1,
                    std::placeholders::_2));
    }

    // start streaming
    if(!realsense->startStreaming()) {
      return EXIT_FAILURE;
    }

    // require a special termination handler to properly close
    shtdown = false;
    signal(SIGINT, [](int) { shtdown = true; });

    // Main loop
    while (true) {
      cv::Mat images, topDebugImage;
      datasetWriter.display(images, topDebugImage);
      if(!images.empty()) {
        cv::imshow("Realsense Images", images);
      }
      int pressed = cv::waitKey(2);
      if(pressed=='q') {
        break;
      }
      if(shtdown) {
        break;
      }
    }
    // Stop the pipeline
    realsense->stopStreaming();

  }

  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
