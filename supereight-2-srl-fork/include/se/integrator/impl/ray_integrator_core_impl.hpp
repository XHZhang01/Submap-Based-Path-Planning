//
// Created by boche on 8/11/22.
//

#ifndef SE_RAY_INTEGRATOR_CORE_IMPL_HPP
#define SE_RAY_INTEGRATOR_CORE_IMPL_HPP

namespace se{

namespace ray_integrator{

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
    size_t observed_count = 0;

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
        if (child_data.observed == true) {
            observed_count++;
        }
    }

    typename NodeT::DataType node_data = node_ptr->getData();

    if (data_count > 0) {
        node_data.occupancy = max_mean_occupancy; // TODO: Need to check update?
        node_data.weight = max_weight;
        if(observed_count == 8) {
            node_data.observed = true;
        }
        node_ptr->setData(node_data);
    }
    return node_data;
}

template<typename BlockT>
inline void propagate_block_to_coarsest_scale(se::OctantBase* octant_ptr)
{
    typedef typename BlockT::DataType DataType;

    BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);

    int origin_scale = block_ptr->getCurrentScale();
    int child_scale = origin_scale;
    int size_at_child_scale_li = BlockT::size >> child_scale;
    int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

    int parent_scale = child_scale + 1;
    block_ptr->setCurrentScale(parent_scale);
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
                size_t observed_count = 0;

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
                                else if (occupancy < min_occupancy) { // ToDo: originally " > " was used here => should change to "<"?, clarify!!
                                    min_data.occupancy = child_data.occupancy;
                                    min_data.weight = child_data.weight;
                                    min_occupancy = occupancy;
                                }
                            }
                            if (child_data.observed) {
                                observed_count++;
                            }

                        } // i
                    }     // j
                }         // k

                if (data_count > 0) {
                    parent_data.occupancy = mean_occupancy / data_count;
                    parent_data.weight = ceil((float) mean_weight) / data_count;
                    parent_data.observed = false;
                    block_ptr->setData(parent_data_idx,parent_data);

                    parent_max_data.occupancy = max_mean_occupancy;
                    parent_max_data.weight = max_weight;
                    if(observed_count==8){
                        parent_max_data.observed = true;
                        parent_data.observed = true;
                    }
                    // ToDo: missing min here?
                }

            } // x
        }     // y
    }         // z



    // Now propagate from parent scale to maximum block scale
    for (parent_scale += 1; parent_scale <= BlockT::getMaxScale(); ++parent_scale) {
        block_ptr->setCurrentScale(parent_scale);
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
                    size_t observed_count = 0;

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
                                if(child_data.observed){
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                        if (data_count > 0) {
                        parent_data.occupancy = mean_occupancy / data_count;
                        parent_data.weight = ceil((float) mean_weight) / data_count;
                        parent_data.observed = false;
                        block_ptr->setData(parent_data_idx,parent_data);

                        parent_max_data.occupancy = max_mean_occupancy;
                        parent_max_data.weight = max_weight;
                        if(observed_count == 8) {
                            parent_max_data.observed = true;
                            parent_data.observed = true;
                        }
                    }

                } // x
            }     // y
        }         // z
    }

    block_ptr->setCurrentScale(origin_scale);
    block_ptr->setMinData(min_data);
}

template<typename BlockT>
inline void propagate_block_to_scale(se::OctantBase* octant_ptr, int desired_scale)
{
    typedef typename BlockT::DataType DataType;

    BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);

    int child_scale = block_ptr->getCurrentScale();
    int size_at_child_scale_li = BlockT::size >> child_scale;
    int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

    int parent_scale = child_scale + 1;
    block_ptr->setCurrentScale(parent_scale); // set to parent level to write data
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
                size_t observed_count = 0;

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
                            if(child_data.observed){
                                observed_count++;
                            }


                        } // i
                    }     // j
                }         // k

                if (data_count > 0) {
                    parent_data.occupancy = mean_occupancy / data_count;
                    parent_data.weight = ceil((float) mean_weight) / data_count;
                    parent_data.observed = false;
                    block_ptr->setData(parent_data_idx,parent_data);

                    parent_max_data.occupancy = max_mean_occupancy;
                    parent_max_data.weight = max_weight;
                    if(observed_count==8){
                        parent_max_data.observed = true;
                        parent_data.observed = true;
                    }
                    // ToDo: missing min here?
                }

            } // x
        }     // y
    }         // z



    // Now propagate from parent scale to maximum block scale
    for (parent_scale += 1; parent_scale <= desired_scale; ++parent_scale) {
        block_ptr->setCurrentScale(parent_scale); // set to parent level to write data
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
                    size_t observed_count = 0;

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
                                if(child_data.observed) {
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                    if (data_count > 0) {
                        parent_data.occupancy = mean_occupancy / data_count;
                        parent_data.weight = ceil((float) mean_weight) / data_count;
                        parent_data.observed = false;
                        block_ptr->setData(parent_data_idx,parent_data);

                        parent_max_data.occupancy = max_mean_occupancy;
                        parent_max_data.weight = max_weight;
                        if(observed_count==8){
                            parent_max_data.observed = true;
                            parent_data.observed = true;
                        }
                    }

                } // x
            }     // y
        }         // z
    }

    block_ptr->setMinData(min_data);
}

template<typename BlockT>
inline void propagate_block_down_to_scale(se::OctantBase* octant_ptr, int desired_scale){
    typedef typename BlockT::DataType DataType;
    BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);

    const int last_scale = block_ptr->getCurrentScale();

    int current_scale = last_scale;
    while(current_scale > desired_scale){
        // down propagation
        block_ptr->allocateDownTo(current_scale-1);
        block_ptr->setCurrentScale(current_scale-1);
        //block_ptr->setInitData(DataType());
        assert(block_ptr->getCurrentScale() == current_scale-1);
        // Now do the down propagation (ToDo: buffer strategies)
        int child_scale = current_scale-1;//block_ptr->getCurrentScale();
        int size_at_child_scale_li = BlockT::size >> child_scale;
        int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

        int parent_scale = current_scale;
        int size_at_parent_scale_li = BlockT::size >> parent_scale;
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

                    if(!parent_data.observed){ // ToDo: this should not occur if last integration scale for this block correct
                        continue;
                    }

                    // Access all 8 children per parent
                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                auto child_data = data_at_child_scale[child_data_idx];

                                child_data.weight = parent_data.weight;
                                child_data.occupancy = parent_data.occupancy;
                                child_data.observed = parent_data.observed;
                                block_ptr->setData(child_data_idx,child_data);

                            } // i
                        }     // j
                    }         // k
                } // x
            }     // y
        }         // z
        current_scale--;

    }
}


} // namespace ray_integrator

} // namespace se

#endif //SE_RAY_INTEGRATOR_CORE_IMPL_HPP
