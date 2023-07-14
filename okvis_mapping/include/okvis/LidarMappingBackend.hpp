//
// Created by boche on 8/12/22.
//

#ifndef INCLUDE_OKVIS_LIDARMAPPINGBACKEND_HPP
#define INCLUDE_OKVIS_LIDARMAPPINGBACKEND_HPP

#include <se/supereight.hpp>
#include <okvis/config_mapping.hpp>

#include <opencv2/opencv.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/kinematics/Transformation.hpp>

namespace okvis {

    class LidarMappingBackend {
    public:
        OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Delete default constructor.
         */
         LidarMappingBackend() = delete;

        /**
         * @brief Constructor.
         */
        LidarMappingBackend(std::string yaml_file)
        : config_(yaml_file), map_(config_.map), sensor_(config_.sensor), integrator_(map_)
        {
        }

        /**
         * @brief Destructor, does nothing.
         */
        virtual ~LidarMappingBackend() {}

        /**
         * @brief Return Body to Lidar Transformation of sensor
         * @return
         */
        Eigen::Matrix4f T_BL() const{
          return sensor_.T_BS;
        }

        /**
         *
         */
        void integrateMeasurement(const Eigen::Vector3d &ray_measurement, okvis::kinematics::Transformation &T_WS, const unsigned int frame);

        /**
         *
         */
        void integrateDepthImage(const std::vector<Eigen::Vector3f>& pointCloud, okvis::kinematics::Transformation &T_WS, const unsigned int frame);

        /**
         *
         */
        void saveMesh(std::string file_prefix);
        void saveVoxelMesh(std::string file_prefix);
        void saveStructure(std::string file_prefix);
        void saveSlice(std::string file_prefix, const Eigen::Vector3d& sliceOrigin);



    private:

        se::Config<se::OccupancyDataConfig, se::LeicaLidarConfig> config_;
        se::OccupancyMap<se::Res::Multi> map_;
        se::LeicaLidar sensor_;
        se::MapIntegrator<se::OccupancyMap<se::Res::Multi>> integrator_;

    };

}

#endif // INCLUDE_OKVIS_LIDARMAPPINGBACKEND_HPP
