//
// Created by boche on 6/7/22.
//

#ifndef SE_MULTIRES_OCCUPANCY_RAY_UPDATER_IMPL_HPP
#define SE_MULTIRES_OCCUPANCY_RAY_UPDATER_IMPL_HPP


namespace se {

namespace ray_updater{

template<typename DataT>
inline bool
weighted_mean_update(DataT& data, const se::field_t sample_value, const se::weight_t max_weight)
{
    data.occupancy = (data.occupancy * data.weight + sample_value) / (data.weight + 1);
    data.weight = std::min((data.weight + 1), max_weight);
    data.observed = true;

    return true;
}

template<typename DataT, typename ConfigT>
inline bool update_voxel(DataT& data,
                         const float range_diff,
                         const float tau,
                         const float three_sigma,
                         const ConfigT config)
{
    float sample_value;

    if (range_diff < -three_sigma) {
        sample_value = config.log_odd_min;
    }
    else if (range_diff < tau / 2) {
        sample_value = std::min(config.log_odd_min
                                - config.log_odd_min / three_sigma * (range_diff + three_sigma),
                                config.log_odd_max);
    }
    else if (range_diff < tau) {
        sample_value = std::min(-config.log_odd_min * tau / (2 * three_sigma), config.log_odd_max);
    }
    else {
        return false;
    }

    return weighted_mean_update(data, sample_value, config.max_weight);
}


template<typename DataT, typename ConfigT>
inline void free_node(DataT& node_data, const ConfigT config)
{
    weighted_mean_update(node_data, config.log_odd_min, config.max_weight);
}



template<typename DataT, typename ConfigT>
inline bool free_voxel(DataT& voxel_data, const ConfigT config)
{
    return weighted_mean_update(voxel_data, config.log_odd_min, config.max_weight);
}



template<typename NodeT, typename BlockT>
inline typename NodeT::DataType propagate_to_parent_node(se::OctantBase* octant_ptr,
                                                         const int frame)
{
    NodeT* node_ptr = static_cast<NodeT*>(octant_ptr);

    node_ptr->setTimeStamp(frame);

    se::field_t max_mean_occupancy = 0;
    se::weight_t max_weight = 0;
    se::field_t max_occupancy = -std::numeric_limits<se::field_t>::max();
    size_t data_count = 0;

    for (int child_idx = 0; child_idx < 8; ++child_idx) {
        se::OctantBase* child_ptr = node_ptr->getChild(child_idx);

        if (!child_ptr) {
            continue;
        }

        const auto& child_data = (child_ptr->isBlock())
                                 ? static_cast<BlockT*>(child_ptr)->getMaxData()
                                 : static_cast<NodeT*>(child_ptr)->getMaxData();
        if (child_data.weight > 0
            && child_data.occupancy * child_data.weight > max_occupancy) // At least 1 integration
        {
            data_count++;
            max_mean_occupancy = child_data.occupancy;
            max_weight = child_data.weight;
            max_occupancy = max_mean_occupancy * max_weight;
        }
    }

    typename NodeT::DataType node_data = node_ptr->getData();

    if (data_count > 0) {
        node_data.occupancy = max_mean_occupancy; // TODO: Need to check update?
        node_data.weight = max_weight;
        node_data.observed = true;
        node_ptr->setData(node_data);
    }
    return node_data;
}

template<typename BlockT>
inline void propagate_block_to_coarsest_scale(se::OctantBase* octant_ptr)
{
    typedef typename BlockT::DataType DataType;

    BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);

    int child_scale = block_ptr->getCurrentScale();
    int size_at_child_scale_li = BlockT::size >> child_scale;
    int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

    int parent_scale = child_scale + 1;
    int size_at_parent_scale_li = BlockT::size >> parent_scale;
    int size_at_parent_scale_sq = se::math::sq(size_at_parent_scale_li);

    DataType min_data;
    se::field_t min_occupancy;

    DataType* max_data_at_parent_scale = block_ptr->blockMaxDataAtScale(parent_scale);
    DataType* data_at_parent_scale = block_ptr->blockDataAtScale(parent_scale);
    DataType* data_at_child_scale = block_ptr->blockDataAtScale(child_scale);

    min_data = data_at_child_scale[0];
    min_occupancy = min_data.occupancy * min_data.weight;

    // Iter over all parent scale data
#pragma omp parallel for collapse(3)
    for (int z = 0; z < size_at_parent_scale_li; z++) {
        for (int y = 0; y < size_at_parent_scale_li; y++) {
            for (int x = 0; x < size_at_parent_scale_li; x++) {
                const int parent_data_idx =
                    x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                auto& parent_data = data_at_parent_scale[parent_data_idx];
                auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                se::field_t mean_occupancy = 0;
                se::weight_t mean_weight = 0;

                se::field_t max_mean_occupancy = 0;
                se::weight_t max_weight = 0;
                se::field_t max_occupancy = -std::numeric_limits<se::field_t>::max();

                size_t data_count = 0;

                // Access all 8 children per parent
                for (int k = 0; k < 2; k++) {
                    for (int j = 0; j < 2; j++) {
                        for (int i = 0; i < 2; i++) {
                            const int child_data_idx = (2 * x + i)
                                                       + (2 * y + j) * size_at_child_scale_li
                                                       + (2 * z + k) * size_at_child_scale_sq;
                            const auto child_data = data_at_child_scale[child_data_idx];

                            if (child_data.weight > 0) {
                                // Update mean
                                data_count++;
                                mean_occupancy += child_data.occupancy;
                                mean_weight += child_data.weight;

                                se::field_t occupancy =
                                    (child_data.occupancy * child_data.weight);

                                if (occupancy > max_occupancy) {
                                    // Update max
                                    max_mean_occupancy = child_data.occupancy;
                                    max_weight = child_data.weight;
                                    max_occupancy = max_mean_occupancy * max_weight;
                                }
                                else if (occupancy > min_occupancy) { // ToDo: originally " > " was used here => schould change to "<"?, clarify!!
                                    min_data.occupancy = child_data.occupancy;
                                    min_data.weight = child_data.weight;
                                    min_occupancy = occupancy;
                                }
                            }

                        } // i
                    }     // j
                }         // k

                if (data_count > 0) {
                    parent_data.occupancy = mean_occupancy / data_count;
                    parent_data.weight = ceil((float) mean_weight) / data_count;

                    parent_max_data.occupancy = max_mean_occupancy;
                    parent_max_data.weight = max_weight;
                    // ToDo: missing min here?
                }

            } // x
        }     // y
    }         // z



    // Now propagate from parent scale to maximum block scale
    for (parent_scale += 1; parent_scale <= BlockT::getMaxScale(); ++parent_scale) {
        size_at_parent_scale_li = BlockT::size >> parent_scale;
        size_at_parent_scale_sq = se::math::sq(size_at_parent_scale_li);

        child_scale = parent_scale - 1;
        size_at_child_scale_li = BlockT::size >> child_scale;
        size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

        DataType* max_data_at_parent_scale = block_ptr->blockMaxDataAtScale(parent_scale);
        DataType* data_at_parent_scale = block_ptr->blockDataAtScale(parent_scale);
        DataType* max_data_at_child_scale = block_ptr->blockMaxDataAtScale(child_scale);
        DataType* data_at_child_scale = block_ptr->blockDataAtScale(child_scale);

#pragma omp parallel for collapse(3)
        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                    se::field_t mean_occupancy = 0;
                    se::weight_t mean_weight = 0;

                    se::field_t max_mean_occupancy = 0;
                    se::weight_t max_weight = 0;
                    se::field_t max_occupancy = -std::numeric_limits<float>::max();

                    size_t data_count = 0;

                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                                           + (2 * y + j) * size_at_child_scale_li
                                                           + (2 * z + k) * size_at_child_scale_sq;
                                const auto child_data = data_at_child_scale[child_data_idx];
                                const auto child_max_data = max_data_at_child_scale[child_data_idx];

                                if (child_max_data.weight > 0) {
                                    // Update mean
                                    data_count++;
                                    mean_occupancy += child_data.occupancy;
                                    mean_weight += child_data.weight;

                                    if ((child_max_data.occupancy * child_max_data.weight)
                                        > max_occupancy) {
                                        // Update max
                                        max_mean_occupancy = child_max_data.occupancy;
                                        max_weight = child_max_data.weight;
                                        max_occupancy = max_mean_occupancy * max_weight;
                                    }
                                }

                            } // i
                        }     // j
                    }         // k

                    if (data_count > 0) {
                        parent_data.occupancy = mean_occupancy / data_count;
                        parent_data.weight = ceil((float) mean_weight) / data_count;

                        parent_max_data.occupancy = max_mean_occupancy;
                        parent_max_data.weight = max_weight;
                    }

                } // x
            }     // y
        }         // z
    }

    block_ptr->setMinData(min_data);
}


}

// Multi-res Occupancy per-ray updater
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
RayUpdater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::RayUpdater(
    MapType& map, const SensorT& sensor, const Eigen::Vector3f& ray,
    const Eigen::Matrix4f& T_WS, const int frame) :
        map_(map),
        octree_(*(map.getOctree())),
        sensor_(sensor),
        ray_(ray),
        T_SW_(se::math::to_inverse_transformation(T_WS)),
        frame_(frame),
        map_res_(map.getRes()),
        config_(map),
        node_set_(octree_.getBlockDepth())
{
}



template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayUpdater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
SensorT>::operator()(se::SingleRayCarverAllocation& allocation_list)
{
    assert(allocation_list.block_list.size() == allocation_list.block_ray_sample_list.size());

#ifndef NDEBUG
std::cout << "[DEBUG] Updating " << allocation_list.node_list.size() << " nodes as free space." << std::endl;
std::cout << "[DEBUG] Updating " << allocation_list.block_list.size() << " blocks." << std::endl;
#endif
TICK("free_node_recurse")
    /// (1) Update Nodes (assuming free space here)
    for (unsigned int i = 0; i < allocation_list.node_list.size(); ++i){
        auto node_ptr = static_cast<NodeType*>(allocation_list.node_list[i]);
        const int depth = octree_.getMaxScale() - std::log2(node_ptr->getSize());
#ifndef NDEBUG
std::cout << "[DEBUG] Updating Node at depth " << depth << std::endl;
#endif
        freeNodeRecurse(allocation_list.node_list[i], depth);
    }
TOCK("free_node_recurse")

    /// (2) Update Blocks

TICK("updateBlock")
#pragma omp parallel for
    for (unsigned int i = 0; i < allocation_list.block_list.size(); ++i) {
        updateBlock(allocation_list.block_list[i], allocation_list.block_ray_sample_list[i], allocation_list.block_scale_list[i]);
    }
    TOCK("updateBlock")

    /// (3) Propagate Blocks

    /// (3a) Updated blocks
TICK("propagateBlocks")
#pragma omp parallel for
    for (unsigned int i = 0; i < allocation_list.block_list.size(); ++i) {
        ray_updater::propagate_block_to_coarsest_scale<BlockType>(allocation_list.block_list[i]);
    }
TOCK("propagateBlocks")
    /// (3b) Free space blocks
    //#pragma omp parallel for
    //for (unsigned int i = 0; i < freed_block_list_.size(); ++i) {
    //    ray_updater::propagate_block_to_coarsest_scale<BlockType>(freed_block_list_[i]);
    //}
TICK("propagateToRoot")
    /// (4) Propagate to Root
    propagateToRoot(allocation_list.block_list);
    TOCK("propagateToRoot")
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayUpdater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
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
                    ray_updater::propagate_to_parent_node<NodeType, BlockType>(octant_ptr, frame_);
                node_set_[d - 1].insert(octant_ptr->getParent());

                // If all nodes free space, delete children and just leave coarser resolution
                if (node_data.observed
                    && node_data.occupancy * node_data.weight
                <= 0.95 * map_.getDataConfig().min_occupancy) {
                octree_.deleteChildren(static_cast<NodeType*>(octant_ptr));
                //std::cout << "deleting children " << std::endl;
                }

                } // if parent
            }     // nodes at depth d
        }         // depth d

    ray_updater::propagate_to_parent_node<NodeType, BlockType>(octree_.getRoot(), frame_);
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayUpdater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::freeNodeRecurse(se::OctantBase* octant_ptr, int depth)
{
    NodeType* node_ptr = static_cast<NodeType*>(octant_ptr);

    if (node_ptr->getChildrenMask() == 0) {
        typename NodeType::DataType node_data = node_ptr->getData();
        ray_updater::free_node(node_data, map_.getDataConfig());
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
                child_ptr = octree_.allocateAll(node_ptr, child_idx); // TODO: Can be optimised
            }

            if (child_ptr->isBlock()) {
                freeBlock(child_ptr); // TODO: Add to block_list?
//#pragma omp critical(node_lock)
                { // Add node to node list for later up-propagation (finest node for this tree-branch)
                node_set_[depth - 1].insert(child_ptr->getParent());
                }
//#pragma omp critical(block_lock)
                { // Add node to node list for later up-propagation (finest node for this tree-branch)
                freed_block_list_.push_back(child_ptr);
                }
            }
            else {
            freeNodeRecurse(child_ptr, depth + 1);
            }
        }
    }
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayUpdater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::freeBlock(se::OctantBase* octant_ptr)
{
    BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);

   /* /// Compute the integration scale
    // The last integration scale
    const int last_scale = (block_ptr->getMinScale() == -1) ? 3 : block_ptr->getCurrentScale();
    int integration_scale = last_scale;

    *//*    if(last_scale != 0.)
            std::cout << "last scale not zero" << std::endl;
        if(block_ptr->getMinScale() > -1.)
            std::cout << "getMinScale > -1" << std::endl;*//*
    // check if data has been integrated already
    if (block_ptr->getMinScale() == -1) { // nothing inegrated yet
        // Make sure the block is allocated up to the integration scale
        integration_scale = 0;
        block_ptr->allocateDownTo(integration_scale);
        block_ptr->setCurrentScale(integration_scale);
        block_ptr->setInitData(DataType());
    }
    block_ptr->setMinScale(integration_scale);*/
// ToDo: never really tested and executed
    // Get current scale
    std::cout << "ATTENTION!!!" << std::endl;
    const int last_scale = 0; //block_ptr->getCurrentScale();
    block_ptr->allocateDownTo(last_scale);
    block_ptr->setCurrentScale(last_scale);

    const unsigned int size_at_integration_scale_li = BlockType::size >> last_scale;
    const unsigned int size_at_integration_scale_sq = se::math::sq(size_at_integration_scale_li);

    for (unsigned int z = 0; z < size_at_integration_scale_li; z++) {
        for (unsigned int y = 0; y < size_at_integration_scale_li; y++) {
        //#pragma omp simd // TODO: Move UP
            for (unsigned int x = 0; x < size_at_integration_scale_li; x++) {
                // Update the voxel data based using the depth measurement
                const int voxel_idx =
                    x + y * size_at_integration_scale_li + z * size_at_integration_scale_sq;
                auto& voxel_data = block_ptr->currData(voxel_idx); /// \note pass by reference now.
                ray_updater::free_voxel(voxel_data, map_.getDataConfig());
            } // x
        }     // y
    }         // z
}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayUpdater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,SensorT>
    ::updateBlock(se::OctantBase* octant_ptr, Eigen::Vector3f& ray_sample_point, int desired_scale)
{
    if(ray_sample_point.norm() < 1e-08)
        return;
#ifndef NDEBUG
    std::cout << "[DEBUG] Trying to update block for ray sample point " << ray_sample_point.transpose()
              << " at scale " << desired_scale << std::endl;
#endif
    // Block (pointer) to be updated
    BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
    const int block_size = BlockType::size;
    const Eigen::Vector3i block_coord = block_ptr->getCoord(); /// < Coordinates of block to be updated

    /// Compute the integration scale
    float k_l = 6e-03;
    float res = map_.getRes() + k_l * (ray_.norm()-sensor_.near_plane);
    desired_scale = std::floor( std::log2( res / map_res_) );

    // The last integration scale
    // -- Nothing integrated yet (-1) => set to desired_scale
    // -- otherwise current scale of block ptr
    const int last_scale = (block_ptr->getMinScale() == -1) ? desired_scale : block_ptr->getCurrentScale();
#ifndef NDEBUG
std::cout << "[DEBUG] block_ptr->getCurrentScale(): " << block_ptr->getCurrentScale() << std::endl;
std::cout << "[DEBUG] block_ptr->getMinScale(): " << block_ptr->getMinScale() << std::endl;
std::cout << "[DEBUG] Last scale: " << last_scale << std::endl;
std::cout << "[DEBUG] desired_scale: " << desired_scale << std::endl;
#endif

    int integration_scale = last_scale; // ToDo: what is this for?

    // Case 1: Nothing integrated yet:
    if (block_ptr->getMinScale() == -1) { // nothing inegrated yet
        // Make sure the block is allocated up to the integration scale
        integration_scale = desired_scale;  // ToDo: remove (unneccessary? should be covered by last_scale condition
#ifndef NDEBUG
        std::cout << "[DEBUG] Allocation down to " << integration_scale << std::endl;
        std::cout << "[DEBUG] Updating " << " block at scale: " << integration_scale << std::endl;
#endif
        block_ptr->allocateDownTo(integration_scale);
        //block_ptr->setCurrentScale(integration_scale);
        block_ptr->setInitData(DataType());
    }
    else if (desired_scale < last_scale){
        int current_scale = last_scale;
        while(current_scale > desired_scale){
            // down propagation
            block_ptr->allocateDownTo(current_scale-1);
            // block_ptr->setCurrentScale(integration_scale);
            block_ptr->setInitData(DataType());
            // Now do the down propagation (ToDo: buffer strategies)
            int child_scale = current_scale-1;//block_ptr->getCurrentScale();
            int size_at_child_scale_li = BlockType ::size >> child_scale;
            int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

            int parent_scale = current_scale;
            int size_at_parent_scale_li = BlockType::size >> parent_scale;
            int size_at_parent_scale_sq = se::math::sq(size_at_parent_scale_li);

            DataType* data_at_parent_scale = block_ptr->blockDataAtScale(parent_scale);
            DataType* data_at_child_scale = block_ptr->blockDataAtScale(child_scale);

            // Iter over all parent scale data
            for (int z = 0; z < size_at_parent_scale_li; z++) {
                for (int y = 0; y < size_at_parent_scale_li; y++) {
#pragma omp simd
                    for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];

                    // Access all 8 children per parent
                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                            const int child_data_idx = (2 * x + i)
                                                       + (2 * y + j) * size_at_child_scale_li
                                                       + (2 * z + k) * size_at_child_scale_sq;
                            const auto child_data = data_at_child_scale[child_data_idx];

                            // set child data values to parent values
                            DataType downPropData;
                            downPropData.weight = parent_data.weight;
                            downPropData.occupancy = parent_data.occupancy;
                            block_ptr->setData(child_data_idx,downPropData);
                            //block_ptr->setData(block_coord, current_scale-1, downPropData);



                            } // i
                        }     // j
                    }         // k
                    } // x
                }     // y
            }         // z
            current_scale--;

        }
        // set new scale and min scale
        integration_scale = desired_scale;
    }
    else if (false && desired_scale > last_scale){
        se::ray_updater::propagate_block_to_coarsest_scale<BlockType>(block_ptr);
        // set new scale and min scale
        integration_scale = desired_scale;
        block_ptr->setCurrentScale(integration_scale);
    }


#ifndef NDEBUG
std::cout << "[DEBUG] Desired scale for update: " <<  desired_scale << std::endl;
std::cout << "[DEBUG] Updating " << " block at scale: " << integration_scale << std::endl;
std::cout << "[DEBUG] corresponding coordinates: " << block_coord.transpose() << std::endl;
#endif

    /// (1) Determine block / voxels at desired scale to be updated
    /// Given: Pointer to block, ray sample point in sensor frame, desired scale at which integration
    /// < (a) Get voxel for ray sample
    // Trafo to world frame
    Eigen::Vector3f ray_sample_point_W = (T_SW_.inverse() * ray_sample_point.homogeneous()).head(3);
    // point to voxel
    Eigen::Vector3i voxel_coords;
    map_.pointToVoxel(ray_sample_point_W, voxel_coords);
    // corresponding voxel point
    Eigen::Vector3f voxel_point_W;
    map_.voxelToPoint(voxel_coords, voxel_point_W);
    Eigen::Vector3f voxel_point_S = (T_SW_ * voxel_point_W.homogeneous()).head(3);
    // Project vector to voxel center onto ray

    Eigen::Vector3f projection = (voxel_point_S.dot(ray_sample_point) / ray_sample_point.norm()) * ray_sample_point / ray_sample_point.norm();
    //const float voxel_point_d = voxel_point_S.norm();
    const float voxel_point_d = projection.norm();

    // Compute one tau and 3x sigma value for the block
    float tau =
        compute_tau(ray_.norm(), config_.tau_min, config_.tau_max, map_.getDataConfig());
    //std::cout << "tau = " << tau << std::endl;
    float three_sigma = compute_three_sigma(
        ray_.norm(), config_.sigma_min, config_.sigma_max, map_.getDataConfig());
    //std::cout << "three_sigma = " << three_sigma << std::endl;

    /// < (b) determine voxels to be updated
    // Scale 0: only one voxel
    // Scale 1: determine cube of 2x2x2 voxels on scale 1
    // Scale 2: 4x4x4
    // ...
    const unsigned int integration_stride = 1 << integration_scale; // 1 for scale 0, 2 for scale 1, 4 for scale 2 etc.
    /// < quasi die Anzahl der Voxels pro Richtung, die geupdated werden muss in entsprechender scale
    const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
    ///< Die Größe des Blocks auf der Skala

    const unsigned int size_at_integration_scale_sq = se::math::sq(size_at_integration_scale_li);
    ///< Quadratische Größe des Blocks auf der Skala
    int x0 = block_coord[0] + integration_stride * static_cast<int>((voxel_coords[0]-block_coord[0])/integration_stride);
    int y0 = block_coord[1] + integration_stride * static_cast<int>((voxel_coords[1]-block_coord[1])/integration_stride);
    int z0 = block_coord[2] + integration_stride * static_cast<int>((voxel_coords[2]-block_coord[2])/integration_stride);

    x0 = static_cast<int>((voxel_coords[0]-block_coord[0])/integration_stride);
    y0 = static_cast<int>((voxel_coords[1]-block_coord[1])/integration_stride);
    z0 = static_cast<int>((voxel_coords[2]-block_coord[2])/integration_stride);

    const int voxel_idx = x0 + size_at_integration_scale_li*y0 + size_at_integration_scale_sq*z0;
    auto& voxel_data = block_ptr->currData(voxel_idx);
    float range_diff = ray_sample_point.norm() - ray_.norm();
    ray_updater::update_voxel(voxel_data, range_diff, tau, three_sigma, map_.getDataConfig());


// ToDo: handle case of scale !=1 for the moment enough to consider scale 0
    // ToDO: no loop needed
//    for(unsigned int z = 0; z < integration_stride; z++){
//        for(unsigned int y = 0; y < integration_stride; y++){
//            for(unsigned int x = 0; x < integration_stride; x++){
//                //std::cout << "updating voxel " << Eigen::Vector3i(x0 + x, y0 + y, z0 + z).transpose() << std::endl;
//                //std::cout << Eigen::Vector3i((x0 + x)% 8, (y0 + y)% 8, (z0 + z)% 8).transpose() << std::endl;
//
//                // Compute range difference from voxel and measurement
//                float range_diff = ray_sample_point.norm() - ray_.norm(); // ToDo: do not take voxel_point as base but original ray measurement!
//                // Update the voxel data based using the depth measurement
//                const int voxel_idx = (x0 + x - block_coord[0]) + (y0 + y - block_coord[1])* integration_stride
//                    + (z0 + z - block_coord[2]) * integration_stride*integration_stride;
//                    auto& voxel_data = block_ptr->currData(voxel_idx); /// \note pass by reference now.
//#ifndef NDEBUG
//std::cout << "[DEBUG] Updating " << " block with range diff: " << range_diff << std::endl;
//std::cout << "[DEBUG] ----- Old occupancy value:  " << voxel_data.occupancy << std::endl;
//#endif
//
//                // Update voxel
//                //block_ptr->incrCurrObservedCount(ray_updater::update_voxel(voxel_data, range_diff, tau, three_sigma, map_.getDataConfig()));
//                ray_updater::update_voxel(voxel_data, range_diff, tau, three_sigma, map_.getDataConfig());
//#ifndef NDEBUG
//std::cout << "[DEBUG] ----- New occupancy value:  " << voxel_data.occupancy << std::endl;
//#endif
//
//            }
//        }
//    }
    //block_ptr->incrCurrIntegrCount();
    //ray_updater::propagate_block_to_coarsest_scale<BlockType>(octant_ptr);

}

}


#endif //SE_MULTIRES_OCCUPANCY_RAY_UPDATER_IMPL_HPP
