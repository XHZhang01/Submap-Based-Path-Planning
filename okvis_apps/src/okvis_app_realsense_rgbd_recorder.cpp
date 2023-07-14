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
#include <okvis/DatasetWriterRgbd.hpp>
#include <okvis/RealsenseParameters.hpp>
#include <okvis/RealsenseRgbd.hpp>
#include <okvis/ViParametersReader.hpp>
#include <signal.h>

std::atomic_bool shtdown;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  // command line arguments
  if (argc != 4) {
    LOG(ERROR) << "Usage: ./" << argv[0] << " configuration-yaml camera-confiuguration /path/to/dataset";
    return EXIT_FAILURE;
  }

  try {
    // ToDo -> Only the number of cameras is used in the dataset app. Eliminate the the visensor parameters.
    std::string configFilename(argv[1]);
    okvis::ViParametersReader viParametersReader(configFilename);
    okvis::ViParameters parameters;
    viParametersReader.getParameters(parameters);

    // Get the sensor filename
    std::string realsenseConfigFilename(argv[2]);
    okvis::RealsenseParameters realsenseParams(realsenseConfigFilename);

    // Realsense RGBD sensor. // ToDo-> Sensor type is not used, as both D435 and D455 have similar interface.
    okvis::RealsenseRgbd realsense(okvis::Realsense::SensorType::D435i, realsenseParams);

    // dataset writer
    std::string path(argv[3]);
    okvis::DatasetWriterRgbd datasetWriter(parameters, path, realsenseParams);

    // Connect sensor callbacks to the writer.

    // Imu cb
    realsense.setImuCallback(std::bind(&okvis::DatasetWriterRgbd::addImuMeasurement, &datasetWriter,
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // IR0 and IR1 cb
    realsense.setImagesCallback(
        std::bind(&okvis::DatasetWriterRgbd::addImages, &datasetWriter, std::placeholders::_1, std::placeholders::_2));

    // Depth cb
    realsense.setDepthCallback(std::bind(&okvis::DatasetWriterRgbd::addDepthImage, &datasetWriter,
                                         std::placeholders::_1, std::placeholders::_2));

    // Aligned cb
    realsense.setRGBCallback(std::bind(&okvis::DatasetWriterRgbd::addRGBImage, &datasetWriter, std::placeholders::_1,
                                       std::placeholders::_2));

    // start streaming
    if (!realsense.startStreaming()) {
      return EXIT_FAILURE;
    }

    // require a special termination handler to properly close
    shtdown = false;
    signal(SIGINT, [](int) { shtdown = true; });

    // Main loop
    while (true) {
      datasetWriter.display();
      int pressed = cv::waitKey(2);
      if (pressed == 'q') {
        break;
      }
      if (shtdown) {
        break;
      }
    }
    // Stop the pipeline
    realsense.stopStreaming();

  }

  catch (const rs2::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
              << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
