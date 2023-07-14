//
// Created by boche on 8/12/22.
//

#ifndef INCLUDE_OKVIS_OUSTERMAPPINGBACKEND_HPP
#define INCLUDE_OKVIS_OUSTERMAPPINGBACKEND_HPP

#include <se/supereight.hpp>
#include <okvis/config_mapping.hpp>

#include <opencv2/opencv.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/kinematics/Transformation.hpp>

namespace okvis {

    class OusterMappingBackend{
    public:
        OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Delete default constructor.
         */
        OusterMappingBackend() = delete;

        /**
         * @brief Constructor.
         */
        OusterMappingBackend(std::string yaml_file);

        /**
         * @brief Destructor, does nothing.
         */
        virtual ~OusterMappingBackend() {}

        Eigen::Matrix4f T_BL() const{
          return sensor_.T_BS;
        }

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

        se::Config<se::OccupancyDataConfig, se::OusterLidarConfig> config_;
        se::OccupancyMap<se::Res::Multi> map_;
        se::OusterLidar sensor_;
        se::MapIntegrator<se::OccupancyMap<se::Res::Multi>> integrator_;

        static constexpr int8_t pixel_offset[64] = {
                0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,
                12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18,
                0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18};

    };

}

#endif // INCLUDE_OKVIS_OUSTERMAPPINGBACKEND_HPP
