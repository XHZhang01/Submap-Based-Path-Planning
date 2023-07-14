/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_COLLISION_CHECKER_IMPL_HPP
#define SE_PLANNING_COLLISION_CHECKER_IMPL_HPP

namespace se {
namespace planning {

template<typename KEY_T, typename MapT>
CollisionChecker<KEY_T, MapT>::CollisionChecker(const SubmapMap<KEY_T, MapT>& maps, const float robot_radius_m) :
        maps_(maps), robot_radius_m_(robot_radius_m)
{
    for (const auto& m : maps_) {
        octree_roots_[m.first] = m.second.map->getOctree()->getRoot();
        octree_sizes_[m.first] = m.second.map->getOctree()->getSize();
    }
    assert((robot_radius_m_ >= 0.0f) && "The robot radius must be non-negative");
}



template<typename KEY_T, typename MapT>
bool CollisionChecker<KEY_T, MapT>::isPositionFree(const Eigen::Vector3f& centre_W) const
{   //TODO cambiar por iterador

    for (const auto& it : octree_roots_){
        const Eigen::Vector3f centre_K =
            (maps_[it.first].T_KW * centre_W.homogeneous()).template head<3>();
        if (isPositionFreeImpl(centre_K, *maps_[it.first].map, octree_roots_[it.first], octree_sizes_[it.first])) {
            return true;
        }
    }
    return false;
}



template<typename KEY_T, typename MapT>
bool CollisionChecker<KEY_T, MapT>::isPositionFreeImpl(const Eigen::Vector3f& centre_K,
                                                const MapT& map,
                                                const se::OctantBase* root,
                                                const int root_size) const
{
    const Eigen::Array3f aabb_min_K = map.aabbMin() + robot_radius_m_;
    const Eigen::Array3f aabb_max_K = map.aabbMax() - robot_radius_m_;
    if ((centre_K.array() < aabb_min_K).any() || (centre_K.array() > aabb_max_K).any()) {
        return false;
    }
    // Work in voxel coordinates.
    Eigen::Vector3f centre_v;
    map.template pointToVoxel<Safe::Off>(centre_K, centre_v);
    // Exit quickly if the sphere center is occupied.
    if (!isDataFree(map.getData(centre_K))) {
        return false;
    }
    const float radius_v = robot_radius_m_ / map.getRes();
    // Recursively test the octants that intersect the sphere.
    auto intersects = [&, radius_v](const Eigen::Vector3i& octant_coord, const int octant_size) {
        return octantIntersectsSphere(octant_coord, octant_size, centre_v, radius_v);
    };
    if (!intersects(root->getCoord(), root_size)) {
        return false;
    }
    return isOctantInShapeFree(root, intersects);
}



template<typename KEY_T, typename MapT>
bool CollisionChecker<KEY_T, MapT>::isSegmentFree(const Eigen::Vector3f& start_W,
                                           const Eigen::Vector3f& end_W) const
{   //Iterador
    for (const auto& it : octree_roots_) {
        //Bug here start and end position must be in map coords
        const Eigen::Vector3f start_K = (maps_.at(it.first).T_KW * start_W.homogeneous()).template head<3>();
        const Eigen::Vector3f end_K = (maps_.at(it.first).T_KW * end_W.homogeneous()).template head<3>();
        if (isSegmentFreeImpl(
                start_K, end_K, *maps_.at(it.first).map, octree_roots_.at(it.first), octree_sizes_.at(it.first))) {
            return true;
        }
    }
    return false;
}


template<typename KEY_T, typename MapT>
bool CollisionChecker<KEY_T, MapT>::isSegmentFreeImpl(const Eigen::Vector3f& start_K,
                                               const Eigen::Vector3f& end_K,
                                               const MapT& map,
                                               const se::OctantBase* root,
                                               const int root_size) const
{
    if (!isPositionFreeImpl(end_K, map, root, root_size)) {
        return false;
    }
    // Work in voxel coordinates.
    Eigen::Vector3f start_v, end_v;
    if (!map.pointToVoxel(start_K, start_v) || !map.pointToVoxel(end_K, end_v)) {
        return false;
    }
    // Exit quickly if the sphere center is occupied.
    if (!isDataFree(map.getData((start_K + end_K) / 2))) {
        return false;
    }
    const Eigen::Vector3f centre_v = (start_v + end_v) / 2.0f;
    const Eigen::Vector3f axis_v = (end_v - start_v).normalized();
    const float height_v = (end_v - start_v).norm();
    const float radius_v = robot_radius_m_ / map.getRes();
    // Recursively test the octants that intersect the cylinder.
    auto intersects = [&, height_v, radius_v](const Eigen::Vector3i& octant_coord,
                                              const int octant_size) {
        return octantIntersectsCylinder(
            octant_coord, octant_size, centre_v, axis_v, height_v, radius_v);
    };
    if (!root || !intersects(root->getCoord(), root_size)) {
        return false;
    }
    return isOctantInShapeFree(root, intersects);
}



template<typename KEY_T, typename MapT>
bool CollisionChecker<KEY_T, MapT>::octantIntersectsSphere(const Eigen::Vector3i& octant_coord,
                                                    const int octant_size,
                                                    const Eigen::Vector3f& sphere_centre,
                                                    const float sphere_radius)
{
    // Algorithm from Game Physics Cookbook by Gabor Szauer, ISBN 9781787123663
    const Eigen::Vector3f octant_min = octant_coord.cast<float>();
    const Eigen::Vector3f octant_max =
        (octant_coord + Eigen::Vector3i::Constant(octant_size)).cast<float>();
    // Project the sphere centre on the octant.
    const Eigen::Vector3f p = sphere_centre.cwiseMax(octant_min).cwiseMin(octant_max);
    return (sphere_centre - p).squaredNorm() <= sphere_radius * sphere_radius + 1e-5;
}



template<typename KEY_T, typename MapT>
bool CollisionChecker<KEY_T, MapT>::octantIntersectsCylinder(const Eigen::Vector3i& octant_coord,
                                                      const int octant_size,
                                                      const Eigen::Vector3f& cylinder_centre,
                                                      const Eigen::Vector3f& cylinder_axis,
                                                      const float cylinder_height,
                                                      const float cylinder_radius)
{
    // Define a cylinder-centric coordinate frame C with its origin on the cylinder's centre and its
    // z axis along the cylinder's axis of symmetry.
    //  _____________________
    // |        origin       |
    // |          o----------|--z_axis-->
    // |_____________________|
    //  <--cylinder_height-->
    const Eigen::Vector3f& origin = cylinder_centre;
    const Eigen::Vector3f& z_axis = cylinder_axis;
    // Compute the coordinates of the octant vertices. Subtract the origin to make subsequent
    // computations faster.
    // clang-format off
    constexpr int n = 8;
    static const Eigen::Matrix<float, 3, n> vertex_offsets = (Eigen::Matrix<float, 3, n>() <<
        0, 1, 0, 1, 0, 1, 0, 1,
        0, 0, 1, 1, 0, 0, 1, 1,
        0, 0, 0, 0, 1, 1, 1, 1).finished();
    // clang-format on
    const Eigen::Matrix<float, 3, n> octant_vertices =
        ((octant_size * vertex_offsets).colwise() + (octant_coord.cast<float>() - origin));
    // Project the octant vertices on the z axis of frame C.
    // Used a vectorized version of the following (origin already subtracted from octant_vertices):
    // z[i] = (octant_vertices[i] - origin).dot(z_axis);
    const Eigen::Array<float, 1, n> z =
        octant_vertices.cwiseProduct(z_axis.replicate(1, n)).colwise().sum().array();
    // Return early if the octant is completely outside the infinite slab created by the planes of
    // the cylinder's two disks.
    //  /      \  :<----- planes ------>:
    // / octant \ :_____________________:
    // \        / |        origin       |
    //  \      /  |          o----------|--z_axis-->
    //   \    /   |_____________________|
    //    \  /    :<--cylinder_height-->:
    //     \/     :        slab         :
    if ((z < -cylinder_height / 2.0f).all() || (z > cylinder_height / 2.0f).all()) {
        return false;
    }
    // Define arbitrary x and y axes for frame C.
    const Eigen::Vector3f x_axis =
        (std::fabs(z_axis.x() - z_axis.y()) < 1e-5 ? Eigen::Vector3f(-z_axis.z(), 0, z_axis.x())
                                                   : Eigen::Vector3f(-z_axis.y(), z_axis.x(), 0))
            .normalized();
    const Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    // Project the octant vertices on the x-y plane of frame C.
    Eigen::Matrix<float, 2, n> xy;
    xy.topRows<1>() = octant_vertices.cwiseProduct(x_axis.replicate(1, n)).colwise().sum();
    xy.bottomRows<1>() = octant_vertices.cwiseProduct(y_axis.replicate(1, n)).colwise().sum();
    // Project the octant center on the x-y plane of frame C.
    const Eigen::Vector3f octant_centre =
        octant_coord.cast<float>() + Eigen::Vector3f::Constant(octant_size / 2.0f) - origin;
    const Eigen::Vector2f octant_centre_xy(octant_centre.dot(x_axis), octant_centre.dot(y_axis));
    // Compute the greatest distance from the octant centre to the octant boundary on the x-y plane
    // of frame C.
    const float dist_r = (xy.colwise() - octant_centre_xy).colwise().norm().maxCoeff();
    // Dilate the cylinder's projection on the x-y plane of frame C (a circle) by dist_r and test if
    // the octant's projected center is inside.
    const float r = cylinder_radius + dist_r;
    return octant_centre_xy.squaredNorm() <= r * r + 1e-5;
}



template<typename KEY_T, typename MapT>
bool CollisionChecker<KEY_T, MapT>::isDataFree(const typename MapT::DataType& data) const
{
    // The parent max data will be observed only when all children have been observed.
    return data.observed && se::get_field(data) < free_threshold;
}



template<typename KEY_T, typename MapT>
template<typename IntersectsF>
bool CollisionChecker<KEY_T, MapT>::isOctantInShapeFree(const se::OctantBase* octant,
                                                 IntersectsF intersects_shape) const
{
    assert(octant);
    if (octant->isBlock()) {
        const auto* block = static_cast<const typename MapT::OctreeType::BlockType*>(octant);
        return isBlockInShapeFree(block, block->getCoord(), block->getMaxScale(), intersects_shape);
    }
    else {
        const auto* node = static_cast<const typename MapT::OctreeType::NodeType*>(octant);
        if (isDataFree(node->getMaxData())) {
            return true;
        }
        // The node isn't fully free, test the children intersecting the shape.
        const int child_size = node->getSize() / 2;
        for (int i = 0; i < 8; i++) {
            const Eigen::Vector3i child_coord = node->getChildCoord(i);
            if (!intersects_shape(child_coord, child_size)) {
                // This child node doesn't intersect the shape, skip it.
                continue;
            }
            const se::OctantBase* child = node->getChild(i);
            if (!child) {
                return false;
            }
            if (!isOctantInShapeFree(child, intersects_shape)) {
                return false;
            }
        }
        // Either all children intersecting the shape are free or no children intersect the shape.
        // The latter can happen if the octant-shape intersection test is conservative and can
        // produce false positives. In that case the parent octant may intersect while the children
        // don't. In any case there is no collision with this octant.
        return true;
    }
}



template<typename KEY_T, typename MapT>
template<typename IntersectsF>
bool CollisionChecker<KEY_T, MapT>::isBlockInShapeFree(const typename MapT::OctreeType::BlockType* block,
                                                const Eigen::Vector3i& voxel_coord,
                                                const int scale,
                                                IntersectsF intersects_shape) const
{
    assert(block);
    if (isDataFree(block->getMaxData(voxel_coord, scale))) {
        return true;
    }
    if (scale <= block->getCurrentScale()) {
        // No finer data to test.
        return false;
    }
    // Test the octants at the next lower scale.
    const int child_scale = scale - 1;
    const int child_size = 1 << child_scale;
    for (int i = 0; i < 8; i++) {
        const Eigen::Vector3i child_rel_coord((i & 1) > 0, (i & 2) > 0, (i & 4) > 0);
        const Eigen::Vector3i child_coord = voxel_coord + child_size * child_rel_coord;
        if (!intersects_shape(child_coord, child_size)) {
            // This child octant doesn't intersect the shape, skip it.
            continue;
        }
        if (!isBlockInShapeFree(block, child_coord, child_scale, intersects_shape)) {
            return false;
        }
    }
    // Either all children intersecting the shape are free or no children intersect the shape.
    // The latter can happen if the octant-shape intersection test is conservative and can
    // produce false positives. In that case the parent octant may intersect while the children
    // don't. In any case there is no collision with this octant.
    return true;
}

} // namespace planning
} // namespace se

#endif // SE_PLANNING_COLLISION_CHECKER_IMPL_HPP
