//
// Created by boche on 5/27/22.
//
#include "boost/functional/hash.hpp"
#ifndef SE_RAY_INTEGRATOR_IMPL_HPP
#define SE_RAY_INTEGRATOR_IMPL_HPP

#define PRINT_TIMING 0

namespace se{

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::RayIntegrator(MapType& map,
                    const SensorT& sensor,
                    const Eigen::Vector3f& ray,
                    const Eigen::Matrix4f& T_WS,
                    const int frame) : map_(map), octree_(*(map.getOctree())), sensor_(sensor),
        ray_(ray), T_SW_(se::math::to_inverse_transformation(T_WS)), frame_(frame),
        map_res_(map.getRes()), config_(map), node_set_(octree_.getBlockDepth())
{
    tau_ = compute_tau(ray_.norm(), config_.tau_min, config_.tau_max, map_.getDataConfig());
    three_sigma_ = compute_three_sigma(ray_.norm(), config_.sigma_min, config_.sigma_max, map_.getDataConfig());
    ray_dist_ = ray_.norm();
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::operator()(std::vector<se::OctantBase*>& updated_block_set, std::set<size_t>& updated_block_coordinates)
{
    //TICK("Operator() Pre-Processing")
    /// (0) Get Root Pointer
    se::OctantBase* root_ptr = octree_.getRoot();

    /// (1) Allocate all 8 children first level (if not yet allocated)
    octree_.allocateAll(static_cast<NodeType*>(root_ptr), 0);

    /// (2) Compute free space scale (to determine at which level free space allocation can be stopped)
    int free_space_scale = std::ceil( std::log2(config_.free_space_resolution / map_res_) );
    free_space_scale_ = free_space_scale;

    /// (3) Compute surface thickness tau and extend ray with surface thickness
    //float tau = compute_tau(ray_.norm(), config_.tau_min, config_.tau_max, map_.getDataConfig());
    Eigen::Vector3f extended_ray = (ray_dist_ + tau_) * ray_.normalized();
    Eigen::Vector3f extended_ray_direction = extended_ray.normalized();
    float extended_ray_dist = extended_ray.norm();

    /// (4) Raycasting
    double safe_boundary = 0.2;
    Eigen::Vector3f starting_point = safe_boundary * extended_ray_direction;
    Eigen::Vector3f r_i_S = starting_point;
    Eigen::Vector3f r_i_W;
    Eigen::Matrix4f T_WS = T_SW_.inverse();


    while(r_i_S.norm() < extended_ray_dist) {


        // Compute if in free space
        se::RayState ray_state = computeVariance(ray_dist_, r_i_S.norm());

        r_i_W = (T_WS * r_i_S.homogeneous()).head(3);
        Eigen::Vector3i voxel_coord;
        if (map_.template pointToVoxel<se::Safe::On>(r_i_W, voxel_coord)) { // ToDo: even when r_i_W outside of map, update free space along the ray?

            (*this)(r_i_S, voxel_coord, ray_state, root_ptr, updated_block_set, updated_block_coordinates);
            r_i_S += 0.5*map_res_*(1<<computed_integration_scale_) * extended_ray_direction;
        }
        else
            break;
    }

}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
::updateBlocks(std::map<se::OctantBase*,
                            std::vector<
                                std::tuple<
                                    int, float, Eigen::Vector3i
                                    >>>& blocks_to_be_updated)
{
    // Turn Map into vector and sort for scale
    std::vector<std::pair<se::OctantBase*, std::vector<std::tuple<int, float, Eigen::Vector3i>>>> update_vector;
    for( auto it = blocks_to_be_updated.begin(); it != blocks_to_be_updated.end(); ++it ) {
        update_vector.push_back( std::pair<se::OctantBase*, std::vector<std::tuple<int, float, Eigen::Vector3i>>> (it->first, it->second) );
    }

    #pragma omp parallel for// default(none)
    for(size_t i = 0; i < update_vector.size(); i++){
        for(size_t j = 0; j < update_vector.at(i).second.size(); j++){
            updateBlock(update_vector.at(i).first, std::get<2>(update_vector.at(i).second.at(j)),
                        std::get<0>(update_vector.at(i).second.at(j)),
                        std::get<1>(update_vector.at(i).second.at(j)));
        }
    }
}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
::freeNodesRecurse()
{
    if(free_node_list_.size() > 0)
        std::cout << "freeing nodes" << std::endl;
#pragma omp parallel for default(none)
    for(auto node : free_node_list_){
        //std::cout << "freeing nodes" << std::endl;
        auto node_ptr = static_cast<NodeType*>(node);
        const int depth = octree_.getMaxScale() - std::log2(node_ptr->getSize());
        freeNodeRecurse(node, depth);
    }
}
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::propagateBlocksToCoarsestScale()
{
#pragma omp parallel for default(none)
    for(auto block : updated_block_list_){
        //std::cout << "propagating " << updated_block_list_.size() << " blocks" << std::endl;
        se::ray_integrator::propagate_block_to_coarsest_scale<BlockType>(block);
    }
}
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
::propagateBlocksToCoarsestScale(std::vector<se::OctantBase*>& updated_block_set)
{
#pragma omp parallel for //default(none)
    for(auto block : updated_block_set){
        //std::cout << "propagating " << updated_block_list_.size() << " blocks" << std::endl;
        se::ray_integrator::propagate_block_to_coarsest_scale<BlockType>(block);
    }
}
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::propagateToRoot()
{
    propagateToRoot(updated_block_list_);
}
//template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
//void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
//::propagateToRoot(std::vector<se::OctantBase*>& updated_block_set)
//{
//    propagateToRoot(updated_block_set);
//}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
se::RayState RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::computeVariance(const float measured_depth, const float ray_step_depth)
{
    // Assume worst case scenario -> no multiplication with proj_scale
    const float z_diff = (ray_step_depth - measured_depth);
    const float tau_max =
        compute_tau(measured_depth, config_.tau_min, config_.tau_max, map_.getDataConfig());
    const float three_sigma_min = compute_three_sigma(
        measured_depth, config_.sigma_min, config_.sigma_max, map_.getDataConfig());

    if (z_diff < - three_sigma_min){ // Guaranteed free Space
        return se::RayState::FreeSpace;
    }
    else if (z_diff < tau_max / 2.0){
        return se::RayState::Transition;
    }
    else if (z_diff < tau_max){ // within surface thickness => Occupied ToDo: remove distinction transition occupied
        return se::RayState::Occupied;
    }
    else{ // Behind Surface
        return se::RayState::Undefined;
    }
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
template<class SensorTDummy>
typename std::enable_if_t<std::is_same<SensorTDummy, se::LeicaLidar>::value, void>
    RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
        SensorT>::operator()(const Eigen::Vector3f& ray_sample, const Eigen::Vector3i& voxel_coord, se::RayState rayState, se::OctantBase* octant_ptr, std::vector<se::OctantBase*>& updated_block_set, std::set<size_t>& updated_block_coordinates)
{
    //std::cout << "operator(xxx)\n" ;

    /// (1.a) Determine closest currently allocated octant
    se::OctantBase* finest_octant_ptr =
        se::fetcher::finest_octant<OctreeType>(voxel_coord, 0, octant_ptr); // up to scale 0 meaning down to finest resolution

    /// (1.b) Check if block level is already reached
    if(finest_octant_ptr->isBlock()){

        /// ALREADY BLOCK-LEVEL -> DO update here

        /// (2.a) Determine Integration Scale
        // compute integration scale
        BlockType* block_ptr = static_cast<BlockType*>(finest_octant_ptr);
        // The last integration scale
        const int last_scale = (block_ptr->getMinScale() == -1) ? 0 : block_ptr->getCurrentScale();

        // Compute the point of the block centre in the sensor frame
        const unsigned int block_size = BlockType::size;
        const Eigen::Vector3i block_coord = block_ptr->getCoord();
        Eigen::Vector3f block_centre_point_W;

        map_.voxelToPoint(block_coord, block_size, block_centre_point_W);
        const Eigen::Vector3f block_centre_point_C =
            (T_SW_ * (block_centre_point_W).homogeneous()).head(3);


        // The recommended integration scale
        int computed_integration_scale;
        if(rayState == se::RayState::FreeSpace)
            computed_integration_scale = free_space_scale_;
        else {
            computed_integration_scale = sensor_.computeIntegrationScale(block_centre_point_C,
                                                                         map_res_,
                                                                         last_scale,
                                                                         block_ptr->getMinScale(),
                                                                         block_ptr->getMaxScale());
        }



        /// (2.b) Update Block ToDO: test if faster with caching blocks and update later
        //TICK("updateBlock")
        updateBlock(finest_octant_ptr, const_cast<Eigen::Vector3i&>(voxel_coord), computed_integration_scale, ray_sample.norm());
        //TOCK("updateBlock")
        computed_integration_scale_ = computed_integration_scale;

        size_t hash = 0;
        Eigen::Vector3i coord = finest_octant_ptr->getCoord();

        boost::hash_combine(hash, coord.x());
        boost::hash_combine(hash, coord.y());
        boost::hash_combine(hash, coord.z());
        /// (2.c) Save block for later up-propagation (only save once
        if(updated_block_coordinates.count(hash) == 0){
            updated_block_coordinates.insert(hash);
            updated_block_set.push_back(finest_octant_ptr);
        }

        //blocks_to_be_updated_[finest_octant_ptr].push_back(std::pair<int, Eigen::Vector3f> (computed_integration_scale, ray_sample));

        //blocks_to_be_updated[finest_octant_ptr].push_back(std::tuple<int, float, Eigen::Vector3i>(computed_integration_scale, ray_sample.norm(), const_cast<Eigen::Vector3i&>(voxel_coord)));

        /*if(updated_block_list_.empty())
            updated_block_list_.push_back(finest_octant_ptr);
        else if(updated_block_list_.back() != finest_octant_ptr)
            updated_block_list_.push_back(finest_octant_ptr);*/
    }
    else{
        octree_.allocateAll(static_cast<NodeType*>(finest_octant_ptr), 0);
        return (*this)(ray_sample, voxel_coord, rayState, finest_octant_ptr, updated_block_set, updated_block_coordinates);
    }

    return;
}




template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
::updateBlock(se::OctantBase* octant_ptr, Eigen::Vector3i& voxel_coords, int desired_scale, float sample_dist)
{
    if(sample_dist < 1e-08)
        return;
    // Block (pointer) to be updated
    BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
    const Eigen::Vector3i block_coord = block_ptr->getCoord(); /// < Coordinates of block to be updated


    // The last integration scale
    // -- Nothing integrated yet (-1) => set to desired_scale
    // -- otherwise current scale of block ptr
    const int last_scale = (block_ptr->getMinScale() == -1) ? desired_scale : block_ptr->getCurrentScale();
    int integration_scale = last_scale; // ToDo: what is this for?


    // Case 1: Nothing integrated yet:
    if (block_ptr->getMinScale() == -1) { // nothing inegrated yet
        // Make sure the block is allocated up to the integration scale
        integration_scale = desired_scale;  // ToDo: remove (unneccessary? should be covered by last_scale condition
        block_ptr->allocateDownTo(integration_scale);
        block_ptr->setCurrentScale(integration_scale); // ToDo: needed?
        block_ptr->setInitData(DataType());
    }
    else if (desired_scale < last_scale){
        se::ray_integrator::propagate_block_down_to_scale<BlockType>(block_ptr, desired_scale);

        // set new scale and min scale
        integration_scale = desired_scale;
    }
    else if (desired_scale > last_scale){
        se::ray_integrator::propagate_block_to_scale<BlockType>(block_ptr, desired_scale);
        integration_scale = desired_scale;
        //block_ptr->setCurrentScale(integration_scale); ToDo: needed?

    }

    /// < (b) determine voxels to be updated
    const unsigned int integration_stride = 1 << integration_scale; // 1 for scale 0, 2 for scale 1, 4 for scale 2 etc.
    /// < quasi die Anzahl der Voxels pro Richtung, die geupdated werden muss in entsprechender scale
    const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
    ///< Die Größe des Blocks auf der Skala

    const unsigned int size_at_integration_scale_sq = se::math::sq(size_at_integration_scale_li);
    ///< Quadratische Größe des Blocks auf der Skala

    int x0 = static_cast<int>((voxel_coords[0]-block_coord[0])/integration_stride);
    int y0 = static_cast<int>((voxel_coords[1]-block_coord[1])/integration_stride);
    int z0 = static_cast<int>((voxel_coords[2]-block_coord[2])/integration_stride);

    const int voxel_idx = x0 + size_at_integration_scale_li*y0 + size_at_integration_scale_sq*z0;
    DataType* data_at_scale = block_ptr->blockDataAtScale(integration_scale);
    auto& voxel_data = data_at_scale[voxel_idx];
    float range_diff = sample_dist - ray_dist_;
    ray_integrator::update_voxel(voxel_data, range_diff, tau_, three_sigma_, map_.getDataConfig());
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::freeNodeRecurse(se::OctantBase* octant_ptr, int depth)
{
    NodeType* node_ptr = static_cast<NodeType*>(octant_ptr);

    if (node_ptr->getChildrenMask() == 0) {
        typename NodeType::DataType node_data = node_ptr->getData();
        ray_integrator::free_node(node_data, map_.getDataConfig());
        node_ptr->setData(node_data);
        //#pragma omp critical(node_lock)
        { // Add node to node list for later up-propagation (finest node for this tree-branch)
        node_set_[depth - 1].insert(node_ptr->getParent());
        }
    }
    else {
        for (int child_idx = 0; child_idx < 8; child_idx++) {
            se::OctantBase* child_ptr = node_ptr->getChild(child_idx);
            if (!child_ptr) {
                child_ptr = octree_.allocateAll(node_ptr, child_idx); //
            }

            if (child_ptr->isBlock()) {
                freeBlock(child_ptr); // TODO: Add to block_list?
                //#pragma omp critical(node_lock)
                { // Add node to node list for later up-propagation (finest node for this tree-branch)
                    node_set_[depth - 1].insert(child_ptr->getParent());
                }
                //#pragma omp critical(block_lock)
                { // Add node to node list for later up-propagation (finest node for this tree-branch)
                    free_block_list_.push_back(child_ptr);
                }
            }
            else {
                freeNodeRecurse(child_ptr, depth + 1);
            }
        }
    }
}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::freeBlock(se::OctantBase* octant_ptr)
{

    // Block (pointer) to be updated
    BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
    const Eigen::Vector3i block_coord = block_ptr->getCoord(); /// < Coordinates of block to be updated


    // The last integration scale
    // -- Nothing integrated yet (-1) => highest scale 3
    // -- otherwise current scale of block ptr
    const int last_scale = (block_ptr->getMinScale() == -1) ? 3 : block_ptr->getCurrentScale();
    int integration_scale = last_scale; // ToDo: what is this for?

    // Case 1: Nothing integrated yet:
    if (block_ptr->getMinScale() == -1) { // nothing inegrated yet
        block_ptr->allocateDownTo(integration_scale);
        block_ptr->setInitData(DataType());
    }

    const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
    const unsigned int size_at_integration_scale_sq = se::math::sq(size_at_integration_scale_li);
    DataType* data_at_scale = block_ptr->blockDataAtScale(integration_scale);
    for (unsigned int z = 0; z < size_at_integration_scale_li; z++) {
        for (unsigned int y = 0; y < size_at_integration_scale_li; y++) {
            #pragma omp simd // TODO: Move UP
            for (unsigned int x = 0; x < size_at_integration_scale_li; x++) {
            // Update the voxel data based using the depth measurement
                const int voxel_idx =
                    x + y * size_at_integration_scale_li + z * size_at_integration_scale_sq;
                auto& voxel_data = data_at_scale[voxel_idx];

                ray_integrator::free_voxel(voxel_data, map_.getDataConfig());
            } // x
        }     // y
    }         // z
}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
::propagateToRoot(std::vector<se::OctantBase*>& block_list)
{
    // Retrieving Parent Nodes for all updated blocks
    for (const auto& octant_ptr : block_list) {
        BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
        if (block_ptr->getParent()) {
            node_set_[octree_.getBlockDepth() - 1].insert(block_ptr->getParent());
        }
    }

    for (int d = octree_.getBlockDepth() - 1; d > 0; d--) // TODO: block depth - 1?
    {
        std::set<se::OctantBase*>::iterator it;
        for (it = node_set_[d].begin(); it != node_set_[d].end(); ++it) {
            se::OctantBase* octant_ptr = *it;
            if (octant_ptr->getTimeStamp() == frame_) {
                continue;
            }

            if (octant_ptr->getParent()) {
                auto node_data =
                    ray_integrator::propagate_to_parent_node<NodeType, BlockType>(octant_ptr, frame_);
                node_set_[d - 1].insert(octant_ptr->getParent());

                // If all nodes free space, delete children and just leave coarser resolution
                if (node_data.observed
                    && node_data.occupancy * node_data.weight <= 0.95 * map_.getDataConfig().min_occupancy) {
                    octree_.deleteChildren(static_cast<NodeType*>(octant_ptr));
                }

            } // if parent
        }     // nodes at depth d
    }         // depth d

    ray_integrator::propagate_to_parent_node<NodeType, BlockType>(octree_.getRoot(), frame_);
}

}

#endif //SE_RAY_INTEGRATOR_IMPL_HPP
