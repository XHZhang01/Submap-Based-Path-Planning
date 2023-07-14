//
// Created by boche on 8/12/22.
//

#include <okvis/LidarMappingBackend.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
    void LidarMappingBackend::integrateMeasurement(const Eigen::Vector3d &ray_measurement,
                                                   okvis::kinematics::Transformation &T_WS,
                                                   const unsigned int frame) {
      Eigen::Matrix4f T_WL = T_WS.T().template cast<float>() * config_.sensor.T_BS; // for supereight we need body = IMU to {L}idar sensor frame
      integrator_.integrateRay(sensor_, ray_measurement.template cast<float>(), T_WL, frame);
    }

    void LidarMappingBackend::integrateDepthImage(const std::vector<Eigen::Vector3f> &pointCloud,
                                                   okvis::kinematics::Transformation &T_WS, const unsigned int frame) {

      se::Image<float> depth_img(config_.sensor.width, config_.sensor.height);
      Eigen::Vector2i depth_img_resolution(config_.sensor.width, config_.sensor.height);

      // Initialize the image to zeros.
      std::memset(depth_img.data(), 0, depth_img_resolution.prod() * sizeof(float));

      // Read the point cloud.
      std::clog << "Number of points: " << pointCloud.size() << "\n";

      for (auto point : pointCloud) {
        const float_t R = Eigen::Vector3f(point.x(), point.y(), point.z()).norm();
        // project
        if (R < 1.0e-12) {
          continue;
        }

        // Compute azimuth and elevation angles (projection) [deg]
        float rad2deg = 180.0 / M_PI;
        const float_t azimuth = std::atan2(point.x(), point.z()); // ToDo: 2.0 * M_PI - ... ???
        const float_t elevation = std::asin(point.y() / R);
        /*** note that z is in forward direction for BLK2Fly LiDAR sensor
         * This differs from commonly used conventions!
         */


        // Determine row and column in image
        float azimuth_angular_resolution = 360.0f / config_.sensor.width;
        float elevation_angular_resolution = 180.0f / config_.sensor.height;
        float_t u = (azimuth + M_PI)*rad2deg  / azimuth_angular_resolution;
        float_t v = (elevation + 0.5*M_PI)*rad2deg / elevation_angular_resolution;


        // azimuthal wrap-around
        if(u < -0.5){
          u += depth_img_resolution.x();
        }
        if(u > depth_img_resolution.x() - 0.5){
          u -= depth_img_resolution.x();
        }
        if(v < -0.5){
          v += depth_img_resolution.y();
        }
        if(v > depth_img_resolution.y() - 0.5){
          v -= depth_img_resolution.y();
        }

        int row = std::round(v);
        int col = std::round(u);

        if(depth_img(col,row) > 0 ) {
          if (R < depth_img(col, row)) {
            depth_img(col, row) = R;
          }
        }
        else{
          depth_img(col,row) = R;
        }
      }

      std::unique_ptr<uint32_t[]> output_depth_img_data(
              new uint32_t[config_.sensor.width * config_.sensor.height]);
      se::convert_to_output_depth_img(depth_img, config_.sensor.near_plane, config_.sensor.far_plane, output_depth_img_data.get());
      std::vector<cv::Mat> images;
      cv::Size res(config_.sensor.width, config_.sensor.height);
      images.emplace_back(res, CV_8UC4, output_depth_img_data.get());
      cv::imshow("rangeImg", images.at(0));
      cv::waitKey(1);

      Eigen::Matrix4f T_WL = T_WS.T().template cast<float>() * config_.sensor.T_BS; // for supereight we need body = IMU to {L}idar sensor frame
      integrator_.integrateDepth(sensor_, depth_img, T_WL, frame);
    }

    void LidarMappingBackend::saveMesh(std::string file_prefix){
      map_.saveMesh(file_prefix+"mesh.ply");
    }
    void LidarMappingBackend::saveVoxelMesh(std::string file_prefix){
      map_.saveMeshVoxel(file_prefix+"voxelMesh.ply");
    }
    void LidarMappingBackend::saveStructure(std::string file_prefix){
      map_.saveStructure(file_prefix+"struct.ply");
    }
    void LidarMappingBackend::saveSlice(std::string file_prefix, const Eigen::Vector3d& sliceOrigin){
      map_.saveFieldSlices(file_prefix+"slice_x_v.vtk", file_prefix+"slice_y_v.vtk", file_prefix+"slice_z_v.vtk", Eigen::Vector3f(sliceOrigin.x(),sliceOrigin.y(),sliceOrigin.z()));
    }
}
