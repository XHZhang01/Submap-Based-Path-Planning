//
// Created by boche on 5/27/22.
//

#ifndef SE_SINGLE_RAY_CARVER_HPP
#define SE_SINGLE_RAY_CARVER_HPP



#include <Eigen/Core>
#include <set>

#include "se/common/image_utils.hpp"
#include "se/common/str_utils.hpp"
#include "se/integrator/updater/multires_ofusion_core.hpp"
#include "se/map/map.hpp"


namespace se {


enum class RayState { FreeSpace, Transition, Occupied, Undefined };

struct SingleRayCarverAllocation {
    std::vector<se::OctantBase*> node_list;
    std::vector<int> node_scale_list;
    std::vector<se::OctantBase*> block_list;
    std::vector<Eigen::Vector3f> block_ray_sample_list; // Save current sample point along ray for update step
    std::vector<int> block_scale_list; // ToDo might not be required as block scale can be retrieved by accessing current scale
};




template<typename MapT, typename SensorT>
class SingleRayCarver {
    public:
    SingleRayCarver(MapT& /* map */,
                    const SensorT& /* sensor */,
                    const Eigen::Vector3f& /*ray*/,
                    const Eigen::Matrix4f& /* T_SW need Lidar frame?*/,
                    const int /* frame */){};
};

/**
 * \brief Allocate along the ray using ??? a map-to-camera volume carving approach
 *
 * \tparam ColB
 * \tparam SemB
 * \tparam BlockSize
 * \tparam SensorT
 */
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class SingleRayCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                   SensorT> {
    public:
    typedef Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi> MapType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename OctreeType::NodeType NodeType;
    typedef typename OctreeType::BlockType BlockType;

    /**
    * \brief The config file of the single ray carver
    *
    * \param[in] map   The map to allocate the ray in
    */
    struct SingleRayCarverConfig {
        SingleRayCarverConfig(const MapType& map) :
                sigma_min(map.getDataConfig().sigma_min_factor * map.getRes()),
                sigma_max(map.getDataConfig().sigma_max_factor * map.getRes()),
                tau_min(map.getDataConfig().tau_min_factor * map.getRes()),
                tau_max(map.getDataConfig().tau_max_factor * map.getRes()),
                free_space_resolution(map.getFreeSpaceRes())
        {
        }

        const float sigma_min;
        const float sigma_max;
        const float tau_min;
        const float tau_max;
        const float free_space_resolution;
    };


    /**
    * \brief Setup the single ray carver.
    *
    * \param[in]  map                  The reference to the map to be updated.
    * \param[in]  sensor               The sensor model.
    * \param[in]  ray                  The ray to be integrated.
    * \param[in]  T_WS                 The transformation from sensor to world frame.
    * \param[in]  frame                The frame number to be integrated.
    */
    SingleRayCarver(MapType& map,
                 const SensorT& sensor,
                 const Eigen::Vector3f ray,
                 const Eigen::Matrix4f& T_WS,
                 const int frame);

    /**
    * \brief Allocate along the ray using ??? a map-to-camera volume carving approach ToDo refine description
    */
    SingleRayCarverAllocation operator()();



    private:

    /**
    * \brief Return a conservative measure of the expected variance of a sensor model inside a voxel
    *        given its position and depth variance.
    *
    * \param[in] depth_value_min Depth measurement max value inside voxel.
    * \param[in] depth_value_max Depth measurement min value inside voxel.
    * \param[in] node_dist_min_m Minimum node distance along z-axis in meter.
    * \param[in] node_dist_max_m Maximum node distance along z-axis in meter.
    *
    * \return Estimate of the variance
    */
    se::RayState computeVariance(const float measured_depth,
                                 const float ray_step_depth);


    /**
    * \brief Recursively decide if to allocate or terminate a node.
    *
    * \note se::LeicaLidar implementation
    *
    * \tparam SensorTDummy
    * \param[in] voxel_coord    The voxel coordinates of the current sample along the ray
    * \param[in] root_ptr       Starting point for tree traversal
    */
    template<class SensorTDummy = SensorT>
    typename std::enable_if_t<std::is_same<SensorTDummy, se::LeicaLidar>::value, void>
    operator()(const Eigen::Vector3f& ray_sample,
               const Eigen::Vector3i& voxel_coord,
               int desired_scale,
               se::OctantBase* octant_ptr);

    MapType& map_;
    OctreeType& octree_;
    const SensorT& sensor_;
    const Eigen::Vector3f ray_;
    const Eigen::Matrix4f T_SW_;
    const int frame_;
    const float map_res_;
    SingleRayCarverConfig config_;
    const float max_depth_value_;
    const float zero_depth_band_;
    const float size_to_radius_;
    SingleRayCarverAllocation allocation_list_;
};


} // namespace se

#include "impl/single_ray_carver_impl.hpp"

#endif //SE_SINGLE_RAY_CARVER_HPP
