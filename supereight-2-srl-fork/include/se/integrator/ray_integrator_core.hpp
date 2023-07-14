//
// Created by boche on 8/11/22.
//

#ifndef SE_RAY_INTEGRATOR_CORE_HPP
#define SE_RAY_INTEGRATOR_CORE_HPP
namespace se{

namespace ray_integrator {

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

/**
 * \brief Summariese the values from the current integration scale recursively
 *        up to the desired scale.
 *
 * \param[in] block         The block to be updated.
 * \param[in] desired_scale Scale to propagate to
*/
template<typename BlockT>
inline void propagate_block_to_scale(se::OctantBase* octant_ptr, int desired_scale);

/**
 * \brief Propagate down to lower scale at block level.
 *
 * \param[in] block         The block to be updated.
 * \param[in] desired_scale Scale to propagate to
*/
template<typename BlockT>
inline void propagate_block_down_to_scale(se::OctantBase* octant_ptr, int desired_scale);
} // namespace ray_integrator

} // namespace se
#include "impl/ray_integrator_core_impl.hpp"

#endif //SE_RAY_INTEGRATOR_CORE_HPP
