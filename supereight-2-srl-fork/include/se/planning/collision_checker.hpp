/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_COLLISION_CHECKER_HPP
#define SE_PLANNING_COLLISION_CHECKER_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cmath>

#include "se/map/map.hpp"
#include "se/map/octree/fetcher.hpp"
#include "se/map/octree/visitor.hpp"
#include "state.hpp"
#include "submap.hpp"

namespace se {
namespace planning {

// TODO Test all maps in parallel instead of separately. The current test will fail for
// spheres/segments that are partly inside multiple submaps but free in each one. Modify
// isOctantInShapeFree() to test all submaps in parallel.
template<typename KEY_T, typename MapT>
class CollisionChecker {
    public:
    /** Initialize a CollisionChecker with submaps and a robot radius. The robot radius is used in all
     * subsequent tests.
     */
    CollisionChecker(const SubmapMap<KEY_T, MapT>& maps, const float robot_radius_m);

    bool isPositionFree(const Eigen::Vector3f& centre_W) const;

    bool isSegmentFree(const Eigen::Vector3f& start_W, const Eigen::Vector3f& end_W) const;

    /** Test if an octant intersects a sphere. This is an accurate test that shouldn't produce any
     * false positives or negatives. It should also work with a sphere_radius of 0.
     *
     * \param[in] octant_coord  The coordinates of the octant's base corner.
     * \param[in] octant_size   The length of the octant's edge in voxels.
     * \param[in] sphere_centre The sphere's centre in voxel coordinates.
     * \param[in] sphere_radius The sphere's radius in voxels.
     * \return Whether the octant intersects the sphere.
     */
    static bool octantIntersectsSphere(const Eigen::Vector3i& octant_coord,
                                       const int octant_size,
                                       const Eigen::Vector3f& sphere_centre,
                                       const float sphere_radius);

    /** Test if an octant intersects a cylinder. This is an accurate test that shouldn't produce any
     * false positives or negatives. It should also work with a cylinder_radius of 0.
     *
     * \param[in] octant_coord    The coordinates of the octant's base corner.
     * \param[in] octant_size     The length of the octant's edge in voxels.
     * \param[in] cylinder_centre The cylinder's centre in voxel coordinates.
     * \param[in] cylinder_axis   The cylinder's axis of symmetry in voxel coordinates.
     * \param[in] cylinder_height The cylinder's height in voxels.
     * \param[in] cylinder_radius The cylinder's radius in voxels.
     * \return Whether the octant intersects the cylinder.
     */
    static bool octantIntersectsCylinder(const Eigen::Vector3i& octant_coord,
                                         const int octant_size,
                                         const Eigen::Vector3f& cylinder_centre,
                                         const Eigen::Vector3f& cylinder_axis,
                                         const float cylinder_height,
                                         const float cylinder_radius);

    static constexpr float free_threshold = -5.0f;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    bool isPositionFreeImpl(const Eigen::Vector3f& centre_K,
                            const MapT& map,
                            const se::OctantBase* root,
                            const int root_size) const;

    bool isSegmentFreeImpl(const Eigen::Vector3f& start_K,
                           const Eigen::Vector3f& end_K,
                           const MapT& map,
                           const se::OctantBase* root,
                           const int root_size) const;

    bool isDataFree(const typename MapT::DataType& data) const;

    template<typename IntersectsF>
    bool isOctantInShapeFree(const se::OctantBase* octant,
                             IntersectsF intersects_shape) const;

    template<typename IntersectsF>
    bool isBlockInShapeFree(const typename MapT::OctreeType::BlockType* block,
                            const Eigen::Vector3i& voxel_coord,
                            const int scale,
                            IntersectsF intersects_shape) const;

    const SubmapMap<KEY_T, MapT>& maps_;
    // std::vector<const se::OctantBase*> octree_roots_;
    // std::vector<int> octree_sizes_;
    std::unordered_map<KEY_T, se::OctantBase*, std::hash<KEY_T>, std::equal_to<KEY_T>> octree_roots_;
    std::unordered_map<KEY_T, int, std::hash<KEY_T>, std::equal_to<KEY_T>> octree_sizes_;
    const float robot_radius_m_;

    static_assert(MapT::DataType::fld_ == se::Field::Occupancy,
                  "se::planning::CollisionChecker is only implemented for occupancy maps");
};

} // namespace planning
} // namespace se

#include "impl/collision_checker_impl.hpp"

#endif // SE_PLANNING_COLLISION_CHECKER_HPP
