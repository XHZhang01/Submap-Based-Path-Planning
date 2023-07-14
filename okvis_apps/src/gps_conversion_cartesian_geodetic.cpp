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
//#include <opencv4/opencv2/viz/core.hppwidgets.hpp>
#include <opencv2/core/core.hpp>

int main(int argc, char **argv){

    // USAGE: ./gps_conversion_cartesian_geodetic [meas / traj] [path_to_csv_file] [output_file_name]

    if(!(argc==4)){
      std::cout << "[ERROR] Wrong Usage. Usage:  ./gps_conversion_cartesian_geodetic [meas / traj] [path_to_csv_file] [output_file_name]" << std::endl;
      return -1;
      }

    GeographicLib::Math::real lat0,lon0,alt0;
    // Self-recorded, run1
    lat0 = 48.2627857L;
    lon0 = 11.6691351L;
    alt0 = 524.937L;

    lat0 = 47.3686498L; // Laengengrad
    lon0 = 8.5391825L; // Breitengrad
    alt0 = 408.0L; // Hoehe (Zuerich Koordinaten)
    // KITTI Sequence28
//    lat0 = 48.985122186805L;
//    lon0 = 8.3939808937919L;
//    alt0 = 116.35473632812L;
    // KITTI Sequence33
    //lat0 = 48.972109387979L;
    //lon0 = 8.4761310645018L;
    //alt0 = 201.80104064941L;




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

    //std::string path(argv[2]);
    //std::string inputFileName = path + "data.csv";
    std::string inputFileName(argv[2]);
    //std::string outputFileName = path + "data_raw.csv";
    std::string outputFileName(argv[3]);


    // open output file
    std::ofstream outputFile(outputFileName);

    if(trajectoryMode){
        outputFile << "#lon, lat, alt \n";
        // different order due to different convention used in KML files
      }
    else if(measurementMode){
        outputFile << "#timestamp , lat, long, alt, hErr, vErr \n";
      }



    // Open record file
    std::ifstream input;
    input.open(inputFileName);
    std::string inputLine;
    std::getline(input, inputLine); // skipping first line
    //OKVIS_ASSERT_TRUE(Exception, record.good(), "Record File not found");

    // Initialise local cartesian frame
    GeographicLib::Math::real longitude,latitude,altitude,x,y,z,hErr,vErr;
    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();

    // Initialize global cartesian frame
    // GeographicLib::Math::real lat0 = 47.3686498L; // Laengengrad
    // GeographicLib::Math::real lon0 = 8.5391825L; // Breitengrad
    // GeographicLib::Math::real alt0 = 408.0L; // Hoehe (Zuerich Koordinaten)
    GeographicLib::LocalCartesian globRefFrame(lat0,lon0,alt0,earth);

    std::cout.precision(100);

    // Now iterate all lines
    while(std::getline(input, inputLine)){
        std::stringstream stream(inputLine);
        std::string s;
        std::string timestring;

        if(trajectoryMode){
            // timestamp
            std::getline(stream, s, ',');
            timestring = s;
            // x
            std::getline(stream,s,','); // separate line by commas
            x = std::stod(s);
            // y
            std::getline(stream,s,','); // separate line by commas
            y = std::stod(s);
            // z
            std::getline(stream,s,','); // separate line by commas
            z = std::stod(s);

            // skip the rest?

            globRefFrame.Reverse(x,y,z,latitude, longitude, altitude);

            std::stringstream ss;
            ss.precision(std::numeric_limits<long double>::digits10);// set output precision
            ss << longitude << ","
               << latitude << ","
               << altitude << std::endl;

            // write to output
            outputFile << ss.str(); //extract string from stream

          }
        else if(measurementMode){
          // timestamp
          std::getline(stream, s, ',');
          timestring = s;
          // x
          std::getline(stream,s,','); // separate line by commas
          x = std::stod(s);
          // y
          std::getline(stream,s,','); // separate line by commas
          y = std::stod(s);
          // z
          std::getline(stream,s,','); // separate line by commas
          z = std::stod(s);
          // sig_x
          std::getline(stream,s,','); // separate line by commas
          hErr = std::stod(s);
          // sig_y
          std::getline(stream,s,','); // separate line by commas
          //hErr = std::stod(s);
          // sig_z
          std::getline(stream,s,','); // separate line by commas
          vErr = std::stod(s);

          globRefFrame.Reverse(x,y,z,latitude, longitude, altitude);

          std::stringstream ss;
          ss.precision(std::numeric_limits<long double>::digits10);// set output precision
          ss << timestring << ","
             << latitude << ","
             << longitude << ","
             << altitude << ","
             << hErr << ","
             << vErr << std::endl;

          // write to output
          outputFile << ss.str(); //extract string from stream

        }

      }

    // close output file
    outputFile.close();

}
