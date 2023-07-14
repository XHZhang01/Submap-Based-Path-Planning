/**
 * @file gps_conversion_geodetic_cartesian.cpp
 * @brief This is an experimental script converting geodetic coordinates (longitude, latitude, altitude) to cartesian coordinates using GeographicLib
 * @author Simon Boche
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <Eigen/Core>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/assert_macros.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

/// for visualisation of points
//#include <opencv4/opencv2/viz/viz3d.hpp>
//#include <opencv4/opencv2/viz/widgets.hpp>
#include <opencv2/core/core.hpp>


int main(int argc, char **argv){

  // USAGE: ./gps_conversion_cartesian_geodetic [meas / traj] [path_to_csv_file] [output_file_name]

    if(!(argc==4)){
      std::cout << "[ERROR] Wrong Usage. Usage:  ./gps_conversion_cartesian_geodetic [meas / traj] [path_to_csv_file] [output_file_name]" << std::endl;
      return -1;
    }

    bool trajectoryMode=false;
    bool measurementMode = false;

    if(strcmp(argv[1], "traj")==0)
      trajectoryMode=true;
    else if(strcmp(argv[1], "meas")==0)
      measurementMode=true;
    else{
      std::cout << "ERROR. Wrong usage: ./gps_conversion_cartesian_geodetic [meas / traj] [path_to_csv_file] [output_file_name]"<< std::endl;
      return -1;
    }

    std::string inputFileName(argv[2]);
    std::string outputFileName(argv[3]);

    // Open record file
    std::ifstream input;
    input.open(inputFileName);
    std::string inputLine;
    std::getline(input, inputLine); // skipping first line

    // Initialise local cartesian frame
    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::Math::real lat0,lon0,alt0;
    // Self-recorded, run1
    //lat0 = 48.2627857L;
    //lon0 = 11.6691351L;
    //alt0 = 524.937L;
    // Initialize global cartesian frame
    //GeographicLib::Math::real lat0 = 47.3686498L; // Laengengrad
    //GeographicLib::Math::real lon0 = 8.5391825L; // Breitengrad
    //GeographicLib::Math::real alt0 = 408.0L; // Hoehe (Zuerich Koordinaten)
    //GeographicLib::LocalCartesian globRefFrame(lat0,lon0,alt0,earth);

    std::cout.precision(100);


    // open output file
    std::ofstream outputFile(outputFileName);
    if(measurementMode)
      outputFile << "#timestamp , p_x , p_y , p_z , sigma_x , sigma_y , sigma_z \n";
    else if(trajectoryMode)
      outputFile << "# p_x , p_y , p_z  \n";
    std::vector<cv::Point3d> gpsMeasurements;
    std::vector<cv::Point3d> gpsMeasurements_inacc;

    // Open record file
    std::ifstream record;
    record.open(inputFileName);
    //OKVIS_ASSERT_TRUE(Exception, record.good(), "Record File not found");

    // Parse File
    std::string recordLine;
    std::getline(record,recordLine); // skip first line (header) => do nothing

    // Initialise
    double longitude,latitude,altitude,x,y,z,hErr,vErr;

    std::getline(record,recordLine); // first data line
    std::stringstream stream0(recordLine); // first line as stream

    //parse record line
    std::string s0, timestring;
    // timestamp
    std::getline(stream0,s0,','); // separate line by commas
    timestring = s0;
    // latitude
    std::getline(stream0,s0,','); // separate line by commas
    latitude = std::stold(s0);
    // longitude
    std::getline(stream0,s0,','); // separate line by commas
    longitude = std::stold(s0);
    // altitude
    std::getline(stream0,s0,','); // separate line by commas
    altitude = std::stold(s0);
    // horizontal accuracy
    std::getline(stream0,s0,','); // separate line by commas
    hErr = std::stold(s0);
    // vertical accuracy
    std::getline(stream0,s0,','); // separate line by commas
    vErr = std::stold(s0);

    // Initialize global cartesian frame
    std::cout << "Initializing global reference frame with longitude " << longitude << " and latitude " << latitude
              << " and altitude " << altitude << std::endl;
    GeographicLib::LocalCartesian globRefFrame(latitude,longitude,altitude,earth);
    globRefFrame.Forward(latitude,longitude,altitude,x,y,z);
    std::cout << "Initialization points have cartesian coordinates: \n"
              << "x: " << x << ", y: " << y << ", z:" << z << std::endl;

    // write to output
    //outputFile << timestring << ","
    //           << x << ","
    //           << y << ","
    //           << z << ","
    //           << hErr << ","
    //           << hErr << ","
    //           << vErr << std::endl;

    //gpsMeasurements.push_back(cv::Point3d(x,y,z));

    // Now iterate all remaining lines

    while(std::getline(record, recordLine)){
        std::stringstream stream(recordLine);
        std::string s;

        if(measurementMode){

          //timestamp
          std::getline(stream, s, ',');
          timestring = s;
          // latitude
          std::getline(stream,s,','); // separate line by commas
          latitude = std::stold(s);
          // longitude
          std::getline(stream,s,','); // separate line by commas
          longitude = std::stold(s);
          // altitude
          std::getline(stream,s,','); // separate line by commas
          altitude = std::stold(s);
          // horizontal accuracy
          std::getline(stream,s,','); // separate line by commas
          hErr = std::stold(s);
          // vertical accuracy
          std::getline(stream,s,','); // separate line by commas
          vErr = std::stold(s);
          globRefFrame.Forward(latitude,longitude,altitude,x,y,z);

          // write to output
          if(hErr < 0.1 && vErr < 0.1){
            outputFile << timestring << ","
                       << x << ","
                       << y << ","
                       << z << ","
                       << hErr << ","
                       << hErr << ","
                       << vErr << std::endl;
          }



          if(hErr < 0.1 && vErr < 0.1){
            gpsMeasurements.push_back(cv::Point3d(x,y,z));
          }
          else{
            gpsMeasurements_inacc.push_back(cv::Point3d(x,y,z));
          }

        }
        else if (trajectoryMode) {

          // longitude
          std::getline(stream, s, ','); // separate line by commas
          longitude = std::stold(s);
          // latitude
          std::getline(stream, s, ','); // separate line by commas
          latitude = std::stold(s);
          // altitude
          std::getline(stream, s, ','); // separate line by commas
          altitude = std::stold(s);
          globRefFrame.Forward(latitude, longitude, altitude, x, y, z);

          // write to output
          outputFile << timestring << ","
                     << x << ","
                     << y << ","
                     << z << std::endl;

          gpsMeasurements.push_back(cv::Point3d(x, y, z));
        }

      }

    // close output file
    outputFile.close();

    // visualise measurements
    /*
    cv::viz::Viz3d window; //creating a Viz window
    //Displaying the Coordinate Origin (0,0,0)
    window.showWidget("coordinate", cv::viz::WCoordinateSystem(10));
    //Displaying the 3D points in green
    window.showWidget("points acc", cv::viz::WCloud(gpsMeasurements, cv::viz::Color::green()));
    //window.showWidget("points inacc", cv::viz::WCloud(gpsMeasurements_inacc, cv::viz::Color::red()));
    window.spin();*/

}
