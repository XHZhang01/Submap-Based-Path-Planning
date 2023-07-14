/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
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
 *  Created on: Nov 11, 2020
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file create_gps_datafile_euroc.cpp
 * @brief This file processes a euroc ground truth datafile and produces a GPS style datafile.
 * @author Simon Boche
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <glog/logging.h>

#include <Eigen/Core>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <random>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void createVisualisation(cv::Mat &img , std::vector<cv::Point3d> trajectory, std::vector<cv::Point3d> measurements){
    int imgWidth = img.size().width;
    int imgHeight = img.size().height;


    // find minimum / maximum coordinates

    double x_min,x_max, y_min,y_max;
    x_min = trajectory.front().x;
    x_max = trajectory.front().x;
    y_min = trajectory.front().y;
    y_max = trajectory.front().y;

    for(auto iter = trajectory.begin() ; iter!= trajectory.end() ; iter++){
        if(iter->x < x_min)
            x_min = iter->x;
        if(iter->x > x_max)
            x_max = iter->x;
        if(iter->y < y_min)
            y_min = iter->y;
        if(iter->y > y_max)
            y_max = iter->y;
    }

    std::cout << "(x_min, y_min): " << x_min << " , " << y_min << std::endl;
    std::cout << "(x_max, y_max): " << x_max << " , " << y_max << std::endl;

    // now iterate points, scale them and draw line
    cv::Point2d prevPoint(0.0,0.0);
    cv::Point2d currPoint(0.0,0.0);
    cv::Point2d currMeas(0.0,0.0);
    int count = 0;

    for(auto iter = trajectory.begin() ; iter!= trajectory.end() ; iter++){
        currPoint.x = (iter->x - x_min) / (x_max-x_min) * imgWidth;
        currPoint.y = (iter->y - y_min) / (y_max-y_min) * imgHeight;
        if(count>0)
            cv::line(img, prevPoint,currPoint,cv::Scalar(255,0,0));
        count+=1;
        prevPoint = currPoint;
    }

    for(auto iter = measurements.begin() ; iter != measurements.end() ; iter++){
        currMeas.x = (iter->x - x_min) / (x_max-x_min) * imgWidth;
        currMeas.y = (iter->y - y_min) / (y_max-y_min) * imgHeight;
        cv::circle(img, currMeas, 1.0, cv::Scalar(0,0,255));
    }

}

int main(/*int argc, char **argv*/)
{

    // const std::string GT_FILE = "/home/parallels/Documents/EuRoC/MH_04_difficult/mav0/state_groundtruth_estimate0/data.csv";
    const std::string GT_FILE = "/home/parallels/Documents/tumvi/dataset-outdoors4_512_16/mav0/mocap0/data.csv";
    const double SIGMA_X = 5e-02;
    const double SIGMA_Y = /*0.5**/SIGMA_X;
    const double SIGMA_Z = /*1.5**/SIGMA_X;
    // const std::string OUT_FOLDER = "/home/parallels/Documents/EuRoC/MH_04_difficult/mav0/gps0/";
    const std::string OUT_FOLDER = "/home/parallels/Documents/tumvi/dataset-outdoors4_512_16/mav0/gps0/";
    const int FRAME_DELTA = 1;
    const double GPS_LOSS = 10.0;
    const double GPS_RETURN = 10.0;

    // ----- GPS Test parameters - Begin
    Eigen::Vector3d r_SA(0.15, 0.1, 0.05);
    // ----- GPS Test parameters - End
    size_t signalCounter = 0; // counter to determine when GPS loss will be registered
    // ----- Visualization stuff initialization - begin -----
    cv::Mat visualization(800, 800, CV_8UC3);
    std::vector<cv::Point3d> fullTrajectoryPoints;
    std::vector<cv::Point3d> gpsPositionMeasurements;
    // ----- Visualization stuff initialization - end   -----

    // Initialize random number generators
    std::default_random_engine genx;
    genx.seed(1);
    std::normal_distribution<double> distributionx(0.0, SIGMA_X);
    std::default_random_engine geny;
    geny.seed(2);
    std::normal_distribution<double> distributiony(0.0, SIGMA_Y);
    std::default_random_engine genz;
    genz.seed(3);
    std::normal_distribution<double> distributionz(0.0, SIGMA_Z);


    // Read in GPS signal (from Ground-Truth Data)
    std::ifstream gpsSignal;
    // open the GPS file
    std::string gline;
    gpsSignal.open(GT_FILE);
    int gnumber_of_lines = 0;
    while (std::getline(gpsSignal, gline))
      ++gnumber_of_lines;
    std::cout << "No. GPS measurements: " << gnumber_of_lines-1 << std::endl;
    if (gnumber_of_lines - 1 <= 0) {
      std::cout << "ERROR: no gps messages present" << std::endl;
      return -1;
    }

    // set reading position to second line
    gpsSignal.clear();
    gpsSignal.seekg(0, std::ios::beg);
    std::getline(gpsSignal, gline);


    // output file name
    std::string outputFileName = OUT_FOLDER + "data.csv";
    std::ofstream outputFile(outputFileName);
    outputFile << "#timestamp , p_x , p_y , p_z , sigma_x , sigma_y , sigma_z \n";

    // placeholder variables for read variables
    okvis::Time tg;
    std::string timestring;
    int count = 0; // counter for gps signal
    double px, py, pz;

    std::string line;
    while(std::getline(gpsSignal,line)){
        signalCounter += 1;
        std::stringstream stream(line);
        std::string s;
        std::getline(stream, s, ',');
        if(s[0] == '#')
            continue;
        timestring = s;

        Eigen::Vector3d pos;
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ',');
          pos[j] = std::stof(s);
        }

        // Ground-Truth Orientation
        Eigen::Quaterniond ori;
        std::getline(stream, s, ',');
        ori.w() = std::stof(s);
        std::getline(stream, s, ',');
        ori.x() = std::stof(s);
        std::getline(stream, s, ',');
        ori.y() = std::stof(s);
        std::getline(stream, s, ',');
        ori.z() = std::stof(s);

        // Apply r_SA (antenna position relative to sensor)
        okvis::kinematics::Transformation T_WS(pos,ori);
        pos += T_WS.C()*r_SA;

        // Disturb GT position to create GPS Signal
        double sigx = distributionx(genx);
        px = pos[0] + sigx;
        double sigy = distributiony(geny);
        py = pos[1] + sigy;
        double sigz = distributionz(genz);
        pz = pos[2] + sigz;
//        std::cout << "disturbed position: " << px << " " << py << " " << pz << std::endl;
//        std::cout << "errors: " << sigx << " " << sigy << " " << sigz << std::endl;

        double progress = static_cast<double>(signalCounter)/static_cast<double>(gnumber_of_lines);

        count+=1;
        if(count == FRAME_DELTA){

            // Save gt position for trajectory
            fullTrajectoryPoints.push_back(cv::Point3d(pos[0] , pos[1] , pos[2]));
            if(progress < GPS_LOSS || progress > GPS_RETURN){
                gpsPositionMeasurements.push_back(cv::Point3d(px , py , pz));
                // write to output file
                outputFile << timestring << ","
                           << px << ","
                           << py << ","
                           << pz << ","
                           << SIGMA_X << ","
                           << SIGMA_Y << ","
                           << SIGMA_Z << std::endl;
            }

            count = 0;

        }


    }

    outputFile.close();

    std::cout << "Created " << gpsPositionMeasurements.size() << " GPS measurements subsampled from " << fullTrajectoryPoints.size() << " trajectory points." << std::endl;
    std::cout << "signalCounter: "<< signalCounter << std::endl;
    std::cout << "gnumber_oof_lines" << gnumber_of_lines << std::endl;
    createVisualisation(visualization, fullTrajectoryPoints, gpsPositionMeasurements);
    cv::imshow("TrajectoryVis", visualization);
    cv::waitKey(0);

    return 0;


}
