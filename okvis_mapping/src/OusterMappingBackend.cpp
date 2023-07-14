//
// Created by boche on 8/12/22.
//

#include <okvis/OusterMappingBackend.hpp>
#include <srl/projection/OusterLidar.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
    OusterMappingBackend::OusterMappingBackend(std::string yaml_file):
            config_(yaml_file), map_(config_.map), sensor_(config_.sensor), integrator_(map_)
            {
            }

    void OusterMappingBackend::integrateDepthImage(const std::vector<Eigen::Vector3f> &pointCloud,
                                                      okvis::kinematics::Transformation &T_WS, const unsigned int frame) {

      se::Image<float> depth_img(config_.sensor.width, config_.sensor.height);
      Eigen::Vector2i depth_img_resolution(config_.sensor.width, config_.sensor.height);

      // Initialize the image to zeros.
      std::memset(depth_img.data(), 0, depth_img_resolution.prod() * sizeof(float));

      // Read the point cloud.
      std::clog << "Number of points: " << pointCloud.size() << "\n";

      // Read the point cloud and convert each point to a distance.
      //size_t pixel_idx = SIZE_MAX;
      //std::vector<float> data(depth_img_resolution.prod(), 0);
      //for (const auto& point : pointCloud) {
      //  data[++pixel_idx] = Eigen::Vector3f(point.x(), point.y(), point.z()).norm();
      //}

      for(const auto& point:pointCloud){
        Eigen::Vector2f imagePoint;
        if(sensor_.model.project(point, &imagePoint) == srl::projection::ProjectionStatus::Successful){
          Eigen::Vector2i pix = se::round_pixel(imagePoint);
          float r = point.norm();
          if(depth_img(pix.x(), pix.y()) < r) {
            depth_img(pix.x(), pix.y()) = r;
          }
        }

      }

      // Reorganize the distances into a depth image.
      //for (int y = 0; y < config_.sensor.height; ++y) {
       // for (int x = 0; x < config_.sensor.width; ++x) {
        //  const int tmp_x = (x + pixel_offset[y]) % config_.sensor.width;
         // const size_t idx = tmp_x * config_.sensor.height + y;
          //depth_img(x, y) = data[idx];
        //}
      //}

      std::unique_ptr<uint32_t[]> output_depth_img_data(
              new uint32_t[config_.sensor.width * config_.sensor.height]);
      se::convert_to_output_depth_img(depth_img, config_.sensor.near_plane, config_.sensor.far_plane, output_depth_img_data.get());
      std::vector<cv::Mat> images;
      cv::Size res(config_.sensor.width, config_.sensor.height);
      images.emplace_back(res, CV_8UC4, output_depth_img_data.get());
      cv::imshow("rangeImg", images.at(0));
      cv::imwrite("rangeImg-frame"+std::to_string(frame)+".png",images.at(0));
      cv::waitKey(1);

      Eigen::Matrix4f T_WL = T_WS.T().template cast<float>() * config_.sensor.T_BS; // for supereight we need body = IMU to {L}idar sensor frame
      integrator_.integrateDepth(sensor_, depth_img, T_WL, frame);
    }

    void OusterMappingBackend::saveMesh(std::string file_prefix){
      map_.saveMesh(file_prefix+"mesh.ply");
    }

    void OusterMappingBackend::saveVoxelMesh(std::string file_prefix){
      map_.saveMeshVoxel(file_prefix+"voxelMesh.ply");
    }

    void OusterMappingBackend::saveStructure(std::string file_prefix){
      map_.saveStructure(file_prefix+"struct.ply");
    }

    void OusterMappingBackend::saveSlice(std::string file_prefix, const Eigen::Vector3d& sliceOrigin){
      map_.saveFieldSlices(file_prefix+"slice_x_v.vtk", file_prefix+"slice_y_v.vtk", file_prefix+"slice_z_v.vtk", Eigen::Vector3f(sliceOrigin.x(),sliceOrigin.y(),sliceOrigin.z()));
    }
}
