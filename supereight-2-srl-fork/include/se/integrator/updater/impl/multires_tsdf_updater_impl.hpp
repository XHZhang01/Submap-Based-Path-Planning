/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
#define SE_MULTIRES_TSDF_UPDATER_IMPL_HPP


namespace se {



// Multi-res TSDF updater
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::Updater(
    MapType& map,
    const SensorT& sensor,
    const se::Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const int frame) :
        map_(map), sensor_(sensor), depth_img_(depth_img), T_WS_(T_WS), frame_(frame), config_(map)
{
}



template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Multi, BlockSize>,
             SensorT>::operator()(std::vector<se::OctantBase*>& block_ptrs)
{
    unsigned int block_size = BlockType::getSize();
    const Eigen::Matrix4f T_SW = se::math::to_inverse_transformation(T_WS_);
    const Octree<Data<se::Field::TSDF, ColB, SemB>, se::Res::Multi, BlockSize>& octree =
        *map_.getOctree();

    auto valid_predicate = [&](float depth_value) { return depth_value >= sensor_.near_plane; };

#pragma omp parallel for
    for (unsigned int i = 0; i < block_ptrs.size(); i++) {
        BlockType* block_ptr = static_cast<BlockType*>(block_ptrs[i]);
        block_ptr->setTimeStamp(frame_);
        Eigen::Vector3i block_coord = block_ptr->getCoord();
        Eigen::Vector3f block_centre_point_W;
        map_.voxelToPoint(block_coord, block_ptr->getSize(), block_centre_point_W);
        const Eigen::Vector3f block_centre_point_S =
            (T_SW * block_centre_point_W.homogeneous()).head(3);
        const int last_curr_scale = block_ptr->getCurrentScale();
        const int lower_curr_scale_limit = last_curr_scale - 1;

        const int curr_scale = std::max(sensor_.computeIntegrationScale(block_centre_point_S,
                                                                        map_.getRes(),
                                                                        last_curr_scale,
                                                                        block_ptr->getMinScale(),
                                                                        block_ptr->getMaxScale()),
                                        lower_curr_scale_limit);

        block_ptr->setMinScale(block_ptr->getMinScale() < 0
                                   ? curr_scale
                                   : std::min(block_ptr->getMinScale(), curr_scale));

        if (curr_scale < last_curr_scale) {
            auto parent_down_funct = [](const OctreeType& /* octree */,
                                        se::OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& data_union) {
                data_union.prop_data.delta_tsdf = data_union.data.tsdf;
                data_union.prop_data.delta_weight = 0;
            };

            auto child_down_funct = [&](const OctreeType& octree,
                                        se::OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& child_data_union,
                                        typename BlockType::DataUnion& parent_data_union) {
                se::field_t delta_tsdf =
                    parent_data_union.data.tsdf - parent_data_union.prop_data.delta_tsdf;

                if (child_data_union.data.weight != 0) {
                    child_data_union.data.tsdf =
                        std::max(child_data_union.data.tsdf + delta_tsdf, se::field_t(-1));
                    child_data_union.data.weight = fminf(
                        child_data_union.data.weight + parent_data_union.prop_data.delta_weight,
                        map_.getDataConfig().max_weight);
                    ;
                    child_data_union.prop_data.delta_weight =
                        parent_data_union.prop_data.delta_weight;
                }
                else {
                    const Eigen::Vector3f child_sample_coord_f =
                        se::get_sample_coord(child_data_union.coord, 1 << child_data_union.scale);
                    int child_scale_returned;
                    auto interp_field_value = se::visitor::getFieldInterp(
                        octree, child_sample_coord_f, child_data_union.scale, child_scale_returned);

                    if (interp_field_value) {
                        child_data_union.data.tsdf = *interp_field_value;
                        child_data_union.data.weight = parent_data_union.data.weight;

                        child_data_union.prop_data.delta_tsdf = child_data_union.data.tsdf;
                        child_data_union.prop_data.delta_weight = 0;
                    }
                }
            };

            se::propagator::propagateBlockDown(octree,
                                               static_cast<se::OctantBase*>(block_ptr),
                                               curr_scale,
                                               child_down_funct,
                                               parent_down_funct);
        }

        block_ptr->setCurrentScale(curr_scale);
        const int stride = 1 << curr_scale;

        Eigen::Vector3f point_base_W;
        map_.voxelToPoint(block_coord, stride, point_base_W);
        const Eigen::Vector3f point_base_S = (T_SW * point_base_W.homogeneous()).head(3);
        const Eigen::Matrix3f point_delta_matrix_S =
            (se::math::to_rotation(T_SW) * map_.getRes() * Eigen::Matrix3f::Identity());

        for (unsigned int i = 0; i < block_size; i += stride) {
            for (unsigned int j = 0; j < block_size; j += stride) {
                for (unsigned int k = 0; k < block_size; k += stride) {
                    // Set voxel coordinates
                    Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(i, j, k);

                    // Set sample point in camera frame
                    Eigen::Vector3f point_S =
                        point_base_S + point_delta_matrix_S * Eigen::Vector3f(i, j, k);

                    if (point_S.norm() > sensor_.farDist(point_S)) {
                        continue;
                    }

                    // Fetch image value
                    float depth_value(0);
                    if (!sensor_.projectToPixelValue(
                            point_S, depth_img_, depth_value, valid_predicate)) {
                        continue;
                    }

                    // Update the TSDF
                    const float m = sensor_.measurementFromPoint(point_S);
                    const se::field_t sdf_value = (depth_value - m) / m * point_S.norm();

                    typename BlockType::DataUnion data_union =
                        block_ptr->getDataUnion(voxel_coord, block_ptr->getCurrentScale());
                    updateVoxel(data_union, sdf_value);
                    block_ptr->setDataUnion(data_union);
                } // k
            }     // j
        }         // i


        auto parent_up_funct = [](typename BlockType::DataUnion& parent_data_union,
                                  typename BlockType::DataType& data_tmp,
                                  const int sample_count) {
            if (sample_count != 0) {
                data_tmp.tsdf /= sample_count;
                data_tmp.weight /= sample_count;
                parent_data_union.data.tsdf = data_tmp.tsdf;
                parent_data_union.prop_data.delta_tsdf = data_tmp.tsdf;
                parent_data_union.data.weight = ceil(data_tmp.weight);
                parent_data_union.prop_data.delta_weight = 0;
            }
            else {
                parent_data_union.data = typename BlockType::DataType();
                parent_data_union.prop_data = typename BlockType::PropDataType();
            }
        };

        auto child_up_funct = [](typename BlockType::DataUnion& child_data_union,
                                 typename BlockType::DataType& data_tmp) {
            if (child_data_union.data.weight != 0) {
                data_tmp.tsdf += child_data_union.data.tsdf;
                data_tmp.weight += child_data_union.data.weight;
                return 1;
            }
            return 0;
        };

        se::propagator::propagateBlockUp(octree,
                                         static_cast<se::OctantBase*>(block_ptr),
                                         curr_scale,
                                         child_up_funct,
                                         parent_up_funct);
    }

    se::propagator::propagateTimeStampToRoot(block_ptrs);
}



template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Multi, BlockSize>,
             SensorT>::updateVoxel(typename BlockType::DataUnion& data_union,
                                   const field_t sdf_value)
{
    if (sdf_value > -config_.truncation_boundary) {
        const se::field_t tsdf_value =
            std::min(se::field_t(1), sdf_value / config_.truncation_boundary);
        data_union.data.tsdf = (data_union.data.tsdf * data_union.data.weight + tsdf_value)
            / (data_union.data.weight + se::weight_t(1));
        data_union.data.tsdf =
            se::math::clamp(data_union.data.tsdf, se::field_t(-1), se::field_t(1));
        data_union.data.weight =
            std::min(data_union.data.weight + se::weight_t(1), map_.getDataConfig().max_weight);
        data_union.prop_data.delta_weight++;
    }
}



} // namespace se

#endif // SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
