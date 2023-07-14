//
// Created by boche on 6/7/22.

#ifndef SE_MULTIRES_OCCUPANCY_RAY_UPDATER_HPP
#define SE_MULTIRES_OCCUPANCY_RAY_UPDATER_HPP





#include "se/map/map.hpp"
#include "se/map/octree/propagator.hpp"
#include "se/sensor/sensor.hpp"




#include "se/sensor/sensor.hpp"



namespace se {


namespace ray_updater {

/**
 * \brief Update the weighted mean log-odd octant occupancy and set the octant to observed.
 *
 * \param[in,out] data     The data in the octant.
 * \param[in] sample_value The sample occupancy to be integrated.
 *
 * \return True(ToDo: /false if the voxel has been observed the first time)
 */
template<typename DataT>
inline bool
weighted_mean_update(DataT& data, const se::field_t sample_value, const se::weight_t max_weight);

/**
 * \brief Update a field with a new measurement, a weighting of 1 is considered for the new measurement.
 *
 * \param[in]     range_diff  The range difference between the voxel sample point and the depth value of the reprojection.
 * \param[in]     tau         The estimated wall thickness.
 * \param[in]     three_sigma The 3x sigma uncertainty.
 * \param[in,out] voxel_data  The reference to the voxel data of the voxel to be updated.
 *
 * \return True/false if the node has been observed the first time
 */
template<typename DataT, typename ConfigT>
inline bool update_voxel(DataT& data,
                         const float range_diff,
                         const float tau,
                         const float three_sigma,
                         const ConfigT config);

/**
 * \brief Reduce the node data by the minimum log-odd occupancy update per iteration.
 *        This function can be used to faster update a octant if it's know that it is in free space.
 *        The aim is to increase computation time by avoiding to compute the sample value from scratch.
 *
 * \param[in,out] node_data The reference to the node data.
 */
template<typename DataT, typename ConfigT>
inline void free_node(DataT& node_data, const ConfigT config);


/**
 * \brief Reduce the node data by the minimum log-odd occupancy update per iteration.
 *        This function can be used to faster update a octant if it's know that it is in free space.
 *        The aim is to increase computation time by avoiding to compute the sample value from scratch.
 *
 * \param[in,out] node_data The reference to the node data.
 */
template<typename DataT, typename ConfigT>
inline bool free_voxel(DataT& voxel_data, const ConfigT config);

/**
 * \brief Propagate a summary of the eight nodes children to its parent
 *
 * \param[in] node        Node to be summariesed
 * \param[in] voxel_depth Maximum depth of the octree
 * \param[in] frame       Current frame
 *
 * \return data Summary of the node
 */
template<typename NodeT, typename BlockT>
inline typename NodeT::DataType propagate_to_parent_node(se::OctantBase* octant_ptr,
                                                         const unsigned int frame);

/**
 * \brief Summariese the values from the current integration scale recursively
 *        up to the block's max scale.
 *
 * \param[in] block         The block to be updated.
 * \param[in] initial_scale Scale from which propagate up voxel values
*/
template<typename BlockT>
inline void propagate_block_to_coarsest_scale(se::OctantBase* octant_ptr);
}

// Multi-res Occupancy updater
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class RayUpdater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT> {
    public:
    typedef Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename MapType::OctreeType::NodeType NodeType;
    typedef typename MapType::OctreeType::BlockType BlockType;


    struct UpdaterConfig {
        UpdaterConfig(const MapType& map) :
            sigma_min(map.getRes() * map.getDataConfig().sigma_min_factor),
            sigma_max(map.getRes() * map.getDataConfig().sigma_max_factor),
            tau_min(map.getRes() * map.getDataConfig().tau_min_factor),
            tau_max(map.getRes() * map.getDataConfig().tau_max_factor)
        {
        }

        const float sigma_min;
        const float sigma_max;
        const float tau_min;
        const float tau_max;
    };

    /**
     * \param[in]  map                  The reference to the map to be updated.
     * \param[in]  sensor               The sensor model.
     * \param[in]  depth_img            The depth image to be integrated.
     * \param[in]  T_WS                 The transformation from sensor to world frame.
     * \param[in]  frame                The frame number to be integrated.
     */
    RayUpdater(MapType& map,
               const SensorT& sensor,
               const Eigen::Vector3f& ray,
               const Eigen::Matrix4f& T_WS,
               const int frame);

    void operator()(se::SingleRayCarverAllocation& allocation_list);

    private:
    /**
   . * \brief Propagate all newly integrated values from the voxel block depth up to the root of the octree.
   . */
    void propagateToRoot(std::vector<se::OctantBase*>& block_list);

    void freeBlock(se::OctantBase* octant_ptr);

    void updateBlock(se::OctantBase* octant_ptr, Eigen::Vector3f& ray_sample_point, int desired_scale);


    /**
     * \brief Recursively reduce all children by the minimum occupancy log-odd for a single integration.
     */
    void freeNodeRecurse(se::OctantBase* octant_ptr, int depth);

    private:
    MapType& map_;
    OctreeType& octree_;
    const SensorT& sensor_;
    const Eigen::Vector3f ray_;
    const Eigen::Matrix4f T_SW_;
    const int frame_;
    const float map_res_;
    const UpdaterConfig config_;
    std::vector<std::set<se::OctantBase*>> node_set_;
    std::vector<se::OctantBase*> freed_block_list_;
};



} // namespace se

#include "impl/multires_occupancy_ray_updater_impl.hpp"


#endif //SE_MULTIRES_OCCUPANCY_RAY_UPDATER_HPP
