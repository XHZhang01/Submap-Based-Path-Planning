/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_IMPL_HPP
#define SE_MAP_IMPL_HPP

#include "se/common/str_utils.hpp"
#include "se/map/algorithms/structure_meshing.hpp"

namespace se {



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(
    const Eigen::Vector3f& dim,
    const float res,
    const float fs_res,
    const se::DataConfig<FldT, ColB, SemB> data_config) :
        dimension_(dim),
        resolution_(res),
        free_space_resolution_(fs_res),
        T_MW_(se::math::to_transformation(Eigen::Vector3f(dim / 2))),
        T_WM_(se::math::to_inverse_transformation(T_MW_)),
        lb_M_(Eigen::Vector3f::Zero()),
        ub_M_(dimension_),
        data_config_(data_config)
{
    corner_rel_steps_ << 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1;
    initialiseOctree();
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(
    const MapConfig& map_config,
    const se::DataConfig<FldT, ColB, SemB> data_config) :
        dimension_(map_config.dim),
        resolution_(map_config.res),
        free_space_resolution_(map_config.fs_res),
        T_MW_(map_config.T_MW),
        T_WM_(se::math::to_inverse_transformation(T_MW_)),
        lb_M_(Eigen::Vector3f::Zero()),
        ub_M_(dimension_),
        data_config_(data_config)
{
    const Eigen::Vector3f t_MW = se::math::to_translation(T_MW_);
    if (t_MW.x() < 0 || t_MW.x() >= dimension_.x() || t_MW.y() < 0 || t_MW.y() >= dimension_.y()
        || t_MW.z() < 0 || t_MW.z() >= dimension_.z()) {
        std::cout << "World origin is outside the map" << std::endl;
    }
    corner_rel_steps_ << 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1;
    initialiseOctree();
}


template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
inline bool
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::contains(const Eigen::Vector3f& point_W) const
{
    const Eigen::Vector3f point_M = (T_MW_ * point_W.homogeneous()).template head<3>();
    return (point_M.x() >= lb_M_.x() && point_M.x() <= ub_M_.x() && point_M.y() >= lb_M_.y()
            && point_M.y() <= ub_M_.y() && point_M.z() >= lb_M_.z() && point_M.z() <= ub_M_.z());
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB>
inline const typename Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::DataType
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getData(const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3i voxel_coord;

    if constexpr (SafeB == Safe::Off) // Evaluate at compile time
    {
        pointToVoxel<Safe::Off>(point_W, voxel_coord);
    }
    else {
        if (!pointToVoxel<Safe::On>(point_W, voxel_coord)) {
            return DataType();
        }
    }

    return se::visitor::getData(*octree_ptr_, voxel_coord);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB>
inline std::optional<se::field_t>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldInterp(const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3f voxel_coord_f;

    if constexpr (SafeB == Safe::Off) // Evaluate at compile time
    {
        pointToVoxel<Safe::Off>(point_W, voxel_coord_f);
    }
    else {
        if (!pointToVoxel<Safe::On>(point_W, voxel_coord_f)) {
            return {};
        }
    }

    return se::visitor::getFieldInterp(*octree_ptr_, voxel_coord_f);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB, Res ResTDummy>
inline typename std::enable_if_t<ResTDummy == Res::Multi, std::optional<se::field_t>>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldInterp(const Eigen::Vector3f& point_W,
                                                             int& returned_scale) const
{
    Eigen::Vector3f voxel_coord_f;

    if constexpr (SafeB == Safe::Off) // Evaluate at compile time
    {
        pointToVoxel<Safe::Off>(point_W, voxel_coord_f);
    }
    else {
        if (!pointToVoxel<Safe::On>(point_W, voxel_coord_f)) {
            return {};
        }
    }

    return se::visitor::getFieldInterp(*octree_ptr_, voxel_coord_f, returned_scale);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB>
inline std::optional<se::field_vec_t>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldGrad(const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3f voxel_coord_f;

    if constexpr (SafeB == Safe::Off) // Evaluate at compile time
    {
        pointToVoxel<Safe::Off>(point_W, voxel_coord_f);
    }
    else {
        if (!pointToVoxel<Safe::On>(point_W, voxel_coord_f)) {
            return {};
        }
    }



    if constexpr (ResT == Res::Multi){
        int scale_returned;
        auto field_grad = se::visitor::getFieldGrad(*octree_ptr_, voxel_coord_f,scale_returned);
        if (field_grad) {
            float scale_resolution = resolution_ * (1 << scale_returned);
            *field_grad /= scale_resolution;
        }
        return field_grad;
    }
    else{
        auto field_grad = se::visitor::getFieldGrad(*octree_ptr_, voxel_coord_f);
        if (field_grad) {
            *field_grad /= resolution_;
        }
        return field_grad;
    }
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
TriangleMesh
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::fieldSlice(const Eigen::Matrix4f& T_WO) const
{
    QuadMesh mesh;
    mesh.reserve(math::sq(octree_ptr_->getSize()));
    const Eigen::Array3f aabb_min = aabbMin();
    const Eigen::Array3f aabb_max = aabbMax();
    for (float y = -dimension_.y(); y < dimension_.y(); y += resolution_) {
        for (float x = -dimension_.x(); x < dimension_.x(); x += resolution_) {
            const Eigen::Vector3f point_O(x, y, 0);
            const Eigen::Vector3f point_W = (T_WO * point_O.homogeneous()).head<3>();
            if ((point_W.array() < aabb_min).any() || (point_W.array() > aabb_max).any()) {
                // Skip points outside the map AABB for speed.
                continue;
            }
            Eigen::Vector3i voxel_coord;
            pointToVoxel<Safe::Off>(point_W, voxel_coord);
            mesh.emplace_back();
            mesh.back().vertexes[0] = point_O;
            mesh.back().vertexes[1] = point_O + Eigen::Vector3f(resolution_, 0, 0);
            mesh.back().vertexes[2] = point_O + Eigen::Vector3f(resolution_, resolution_, 0);
            mesh.back().vertexes[3] = point_O + Eigen::Vector3f(0, resolution_, 0);
            for (auto& v : mesh.back().vertexes) {
                v = (T_WO * v.homogeneous()).head<3>();
            }
            mesh.back().data = se::get_field(se::visitor::getData(*octree_ptr_, voxel_coord));
        }
    }
    return quad_to_triangle_mesh(mesh);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
int Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveFieldSlices(
    const std::string& filename_x,
    const std::string& filename_y,
    const std::string& filename_z,
    const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3i voxel_coord;
    pointToVoxel(point_W, voxel_coord);

    auto get_field_value = [&](const Eigen::Vector3i& coord) {
        return se::get_field(se::visitor::getData(*octree_ptr_, coord));
    };

    if (!filename_x.empty()) {
        se::io::save_3d_slice_vtk(
            filename_x,
            Eigen::Vector3i(voxel_coord.x(), 0, 0),
            Eigen::Vector3i(voxel_coord.x() + 1, octree_ptr_->getSize(), octree_ptr_->getSize()),
            get_field_value);
    }
    if (!filename_y.empty()) {
        se::io::save_3d_slice_vtk(
            filename_y,
            Eigen::Vector3i(0, voxel_coord.y(), 0),
            Eigen::Vector3i(octree_ptr_->getSize(), voxel_coord.y() + 1, octree_ptr_->getSize()),
            get_field_value);
    }
    if (!filename_z.empty()) {
        se::io::save_3d_slice_vtk(
            filename_z,
            Eigen::Vector3i(0, 0, voxel_coord.z()),
            Eigen::Vector3i(octree_ptr_->getSize(), octree_ptr_->getSize(), voxel_coord.z() + 1),
            get_field_value);
    }
    return 0;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Field FldTDummy>
typename std::enable_if_t<FldTDummy == se::Field::Occupancy, int>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMaxFieldSlices(const std::string& filename_x,
                                                                 const std::string& filename_y,
                                                                 const std::string& filename_z,
                                                                 const Eigen::Vector3f& point_W,
                                                                 const int scale) const
{
    Eigen::Vector3i voxel_coord;
    pointToVoxel(point_W, voxel_coord);

    auto get_max_field_value = [&](const Eigen::Vector3i& coord) {
        return get_field(se::visitor::getMaxData(*octree_ptr_, coord, scale));
    };

    if (!filename_x.empty()) {
        se::io::save_3d_slice_vtk(
            filename_x,
            Eigen::Vector3i(voxel_coord.x(), 0, 0),
            Eigen::Vector3i(voxel_coord.x() + 1, octree_ptr_->getSize(), octree_ptr_->getSize()),
            get_max_field_value);
    }
    if (!filename_y.empty()) {
        se::io::save_3d_slice_vtk(
            filename_y,
            Eigen::Vector3i(0, voxel_coord.y(), 0),
            Eigen::Vector3i(octree_ptr_->getSize(), voxel_coord.y() + 1, octree_ptr_->getSize()),
            get_max_field_value);
    }
    if (!filename_z.empty()) {
        se::io::save_3d_slice_vtk(
            filename_z,
            Eigen::Vector3i(0, 0, voxel_coord.z()),
            Eigen::Vector3i(octree_ptr_->getSize(), octree_ptr_->getSize(), voxel_coord.z() + 1),
            get_max_field_value);
    }
    return 0;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Res ResTDummy>
typename std::enable_if_t<ResTDummy == Res::Multi, int>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveScaleSlices(const std::string& filename_x,
                                                              const std::string& filename_y,
                                                              const std::string& filename_z,
                                                              const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3i voxel_coord;
    pointToVoxel(point_W, voxel_coord);

    se::OctantBase* root_ptr = octree_ptr_->getRoot();
    const int max_scale = octree_ptr_->getMaxScale();

    auto get_scale = [&](const Eigen::Vector3i& coord) {
        const se::OctantBase* leaf_ptr = se::fetcher::template leaf<OctreeType>(coord, root_ptr);

        if (!leaf_ptr) {
            return max_scale;
        }
        else if (leaf_ptr->isBlock()) {
            const typename OctreeType::BlockType* block_ptr =
                static_cast<const typename OctreeType::BlockType*>(leaf_ptr);
            return block_ptr->getCurrentScale();
        }
        else {
            const typename OctreeType::NodeType* node_ptr =
                static_cast<const typename OctreeType::NodeType*>(leaf_ptr);
            return (node_ptr->isLeaf()) ? se::octantops::size_to_scale(node_ptr->getSize()) : -1;
        }
    };

    if (!filename_x.empty()) {
        se::io::save_3d_slice_vtk(
            filename_x,
            Eigen::Vector3i(voxel_coord.x(), 0, 0),
            Eigen::Vector3i(voxel_coord.x() + 1, octree_ptr_->getSize(), octree_ptr_->getSize()),
            get_scale);
    }
    if (!filename_y.empty()) {
        se::io::save_3d_slice_vtk(
            filename_y,
            Eigen::Vector3i(0, voxel_coord.y(), 0),
            Eigen::Vector3i(octree_ptr_->getSize(), voxel_coord.y() + 1, octree_ptr_->getSize()),
            get_scale);
    }
    if (!filename_z.empty()) {
        se::io::save_3d_slice_vtk(
            filename_z,
            Eigen::Vector3i(0, 0, voxel_coord.z()),
            Eigen::Vector3i(octree_ptr_->getSize(), octree_ptr_->getSize(), voxel_coord.z() + 1),
            get_scale);
    }
    return 0;
}

template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
int Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveStructure(const std::string& filename,
                                                                const Eigen::Matrix4f& T_WM) const
{
    const QuadMesh mesh = octree_structure_mesh(*octree_ptr_);
    if (str_utils::ends_with(filename, ".ply")) {
        return io::save_mesh_ply(mesh, filename, T_WM);
    }
    else if (str_utils::ends_with(filename, ".vtk")) {
        return io::save_mesh_vtk(mesh, filename, T_WM);
    }
    else if (str_utils::ends_with(filename, ".obj")) {
        return io::save_mesh_obj(mesh, filename, T_WM);
    }
    else {
        std::cerr << "Error saving mesh: unknown file extension in " << filename << "\n";
        return 2;
    }
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
int Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMesh(const std::string& filename,
                                                           const Eigen::Matrix4f& T_OW) const
{
    se::TriangleMesh mesh;
    if constexpr (ResT == se::Res::Single) {
        se::algorithms::marching_cube(*octree_ptr_, mesh);
    }
    else {
        se::algorithms::dual_marching_cube(*octree_ptr_, mesh);
    }

    Eigen::Matrix4f T_WM_scale = T_WM_;
    T_WM_scale.topLeftCorner<3, 3>() *= resolution_;
    const Eigen::Matrix4f T_OM = T_OW * T_WM_scale;

    if (str_utils::ends_with(filename, ".ply")) {
        return io::save_mesh_ply(mesh, filename, T_OM);
    }
    else if (str_utils::ends_with(filename, ".vtk")) {
        return io::save_mesh_vtk(mesh, filename, T_OM);
    }
    else if (str_utils::ends_with(filename, ".obj")) {
        return io::save_mesh_obj(mesh, filename, T_OM);
    }
    else {
        std::cerr << "Error saving mesh: unknown file extension in " << filename << "\n";
        return 2;
    }
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
int Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMeshVoxel(const std::string& filename) const
{
    se::TriangleMesh mesh;
    if constexpr (ResT == se::Res::Single) {
        se::algorithms::marching_cube(*octree_ptr_, mesh);
    }
    else {
        se::algorithms::dual_marching_cube(*octree_ptr_, mesh);
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    if (str_utils::ends_with(filename, ".ply")) {
        return io::save_mesh_ply(mesh, filename, T);
    }
    else if (str_utils::ends_with(filename, ".vtk")) {
        return io::save_mesh_vtk(mesh, filename, T);
    }
    else if (str_utils::ends_with(filename, ".obj")) {
        return io::save_mesh_obj(mesh, filename, T);
    }
    else {
        std::cerr << "Error saving mesh: unknown file extension in " << filename << "\n";
        return 2;
    }
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
inline void
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToPoint(const Eigen::Vector3i& voxel_coord,
                                                           Eigen::Vector3f& point_W) const
{
    point_W =
        (T_WM_ * ((voxel_coord.cast<float>() + sample_offset_frac) * resolution_).homogeneous())
            .template head<3>();
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
inline void
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToPoint(const Eigen::Vector3i& voxel_coord,
                                                           const int stride,
                                                           Eigen::Vector3f& point_W) const
{
    point_W =
        (T_WM_
         * ((voxel_coord.cast<float>() + stride * sample_offset_frac) * resolution_).homogeneous())
            .template head<3>();
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
inline void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToCornerPoints(
    const Eigen::Vector3i& voxel_coord,
    Eigen::Matrix<float, 3, 8>& corner_points_W) const
{
    Eigen::Matrix<float, 3, 8> corner_points_M =
        (corner_rel_steps_.colwise() + voxel_coord.cast<float>()) * resolution_;
    corner_points_W = (T_WM_ * corner_points_M.colwise().homogeneous()).topRows(3);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
inline void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToCornerPoints(
    const Eigen::Vector3i& voxel_coord,
    const int stride,
    Eigen::Matrix<float, 3, 8>& corner_points_W) const
{
    Eigen::Matrix<float, 3, 8> corner_points_M =
        ((stride * corner_rel_steps_).colwise() + voxel_coord.cast<float>()) * resolution_;
    corner_points_W = (T_WM_ * corner_points_M.colwise().homogeneous()).topRows(3);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_W,
                                                           Eigen::Vector3i& voxel_coord) const
{
    if (!contains(point_W)) {
        voxel_coord = Eigen::Vector3i::Constant(-1);
        return false;
    }
    voxel_coord =
        (((T_MW_ * point_W.homogeneous()).template head<3>()) / resolution_).template cast<int>();
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_W,
                                                           Eigen::Vector3i& voxel_coord) const
{
    voxel_coord =
        (((T_MW_ * point_W.homogeneous()).template head<3>()) / resolution_).template cast<int>();
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_W,
                                                           Eigen::Vector3f& voxel_coord_f) const
{
    if (!contains(point_W)) {
        voxel_coord_f = Eigen::Vector3f::Constant(-1);
        return false;
    }
    voxel_coord_f = ((T_MW_ * point_W.homogeneous()).template head<3>()) / resolution_;
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_W,
                                                           Eigen::Vector3f& voxel_coord_f) const
{
    voxel_coord_f = ((T_MW_ * point_W.homogeneous()).template head<3>()) / resolution_;
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointsToVoxels(
    const std::vector<Eigen::Vector3f>& points_W,
    std::vector<Eigen::Vector3i>& voxel_coords) const
{
    bool all_valid = true;

    for (auto point_W : points_W) {
        Eigen::Vector3i voxel_coord;
        if (pointToVoxel<SafeB>(point_W, voxel_coord)) {
            voxel_coords.push_back(voxel_coord);
        }
        else {
            all_valid = false;
        }
    }
    return all_valid;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointsToVoxels(
    const std::vector<Eigen::Vector3f>& points_W,
    std::vector<Eigen::Vector3i>& voxel_coords) const
{
    for (auto point_W : points_W) {
        Eigen::Vector3i voxel_coord;
        pointToVoxel<SafeB>(point_W, voxel_coord);
        voxel_coords.push_back(voxel_coord);
    }
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
Eigen::Array3f Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::aabbMin() const
{
    Eigen::Vector3f aabb_min_W_;
    voxelToPoint(octree_ptr_->aabbMin().matrix(), aabb_min_W_);
    return (aabb_min_W_ - resolution_ * sample_offset_frac).array();
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
Eigen::Array3f Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::aabbMax() const
{
    Eigen::Vector3f aabb_max_W_;
    voxelToPoint(octree_ptr_->aabbMax().matrix(), aabb_max_W_);
    return (aabb_max_W_ + resolution_ * sample_offset_frac).array();
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
bool Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::initialiseOctree()
{
    if (octree_ptr_ != nullptr) {
        std::cerr << "Octree has already been initialised\n";
        return false;
    }

    const float max_dim = dimension_.maxCoeff();
    const int max_size = ceilf(max_dim / resolution_);
    const int oct_size = math::power_two_up(max_size);
    octree_ptr_ = std::shared_ptr<se::Octree<DataType, ResT, BlockSize>>(
        new se::Octree<DataType, ResT, BlockSize>(oct_size));
    return true;
}



} // namespace se

#endif // SE_MAP_IMPL_HPP
