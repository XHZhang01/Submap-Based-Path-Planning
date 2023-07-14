//
// Created by boche on 8/9/22.
//
#ifndef SE_RAY_INTEGRATOR_HPP
#define SE_RAY_INTEGRATOR_HPP

#include <Eigen/Core>
#include <set>
#include <thread>
//#include <srl/queue/threadsafe/ThreadsafeQueue.hpp>

#include "se/integrator/ray_integrator_core.hpp"

namespace se{

enum class RayState { FreeSpace, Transition, Occupied, Undefined };

template<typename MapT, typename SensorT>
class RayIntegrator {
    public:
    RayIntegrator(MapT& /* map */,
                    const SensorT& /* sensor */,
                    const Eigen::Vector3f& /*ray*/,
                    const Eigen::Matrix4f& /* T_SW need Lidar frame?*/,
                    const int /* frame */){};
};

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT> {
    public:

    typedef Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename MapType::OctreeType::NodeType NodeType;
    typedef typename MapType::OctreeType::BlockType BlockType;

    /**
    * \brief The config file of the single ray carver
    *
    * \param[in] map   The map to allocate the ray in
    */
    struct RayIntegratorConfig {
        RayIntegratorConfig(const MapType& map) :
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
    RayIntegrator(MapType& map,
                  const SensorT& sensor,
                  const Eigen::Vector3f& ray,
                  const Eigen::Matrix4f& T_WS,
                  const int frame);

    /**
    * \brief Allocate along the ray using ??? a map-to-camera volume carving approach ToDo refine description
    */
    void operator()(std::vector<se::OctantBase*>& updated_block_set, std::set<size_t>& updated_block_coordinates);

    /**
     * Update Operations
     */

    void updateBlocks(std::map<se::OctantBase*,
                               std::vector<
                                   std::tuple<
                                       int, float, Eigen::Vector3i
                                       >>>& blocks_to_be_updated);
    void propagateToRoot();
    void propagateToRoot(std::vector<se::OctantBase*>& block_list);
    //void propagateToRoot(std::vector<se::OctantBase*>& updated_block_set);
    void propagateBlocksToCoarsestScale();
    void propagateBlocksToCoarsestScale(std::vector<se::OctantBase*>& updated_block_set);
    void freeNodesRecurse();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
               //int desired_scale,
               se::RayState rayState,
               se::OctantBase* octant_ptr,
               std::vector<se::OctantBase*>& updated_block_set,
               std::set<size_t>& updated_block_coordinates);


    /**
     * Update Operations
     */

    void updateBlock(se::OctantBase* octant_ptr, Eigen::Vector3i& voxel_coords, int desired_scale, float sample_dist);

    void freeBlock(se::OctantBase* octant_ptr);

    void freeNodeRecurse(se::OctantBase* octant_ptr, int depth);



    MapType& map_;
    OctreeType& octree_;
    const SensorT& sensor_;
    const Eigen::Vector3f ray_;
    const Eigen::Matrix4f T_SW_;
    const int frame_;
    const float map_res_;
    RayIntegratorConfig config_;

    std::vector<std::set<se::OctantBase*>> node_set_;
    std::vector<se::OctantBase*> free_block_list_;
    std::vector<se::OctantBase*> updated_block_list_;
    //std::vector<std::tuple<se::OctantBase*, Eigen::Vector3f, int>> blocks_to_be_updated_; /// [block to be updated -> sample point + desired integration scale]
    std::vector<se::OctantBase*> free_node_list_;

    int free_space_scale_ = 0;
    int computed_integration_scale_=0;

    // Sensor Model - Tau and Sigma
    float tau_ = 0.;
    float three_sigma_ = 0.;
    float ray_dist_ = 0.;

    // Book-Keeping for late updating
    std::map<se::OctantBase*, std::vector<std::pair<int, Eigen::Vector3f>>> blocks_to_be_updated_; ///< Map BlockPtr -> pairs (integration_scale, ray_point)

};

} // namespace se
#include "impl/ray_integrator_impl.hpp"

#endif // SE_RAY_INTEGRATOR_HPP