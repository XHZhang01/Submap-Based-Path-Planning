/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_SUBMAP_HPP
#define SE_PLANNING_SUBMAP_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>
#include <unordered_map>

#include "se/common/math_util.hpp"

namespace se {
namespace planning {

template<typename MapT>
struct Submap {
    const MapT* const map;
    const Eigen::Matrix4f T_WK;
    const Eigen::Matrix4f T_KW;

    Submap(const MapT* const map = nullptr,
           const Eigen::Matrix4f& T_WK = Eigen::Matrix4f::Identity()) :
            map(map), T_WK(T_WK), T_KW(math::to_inverse_transformation(T_WK))
    {
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename MapT>
using SubmapVec = std::vector<Submap<MapT>, Eigen::aligned_allocator<Submap<MapT>>>;
template<typename KEY_T, typename MapT>
using SubmapMap = std::unordered_map<KEY_T, Submap<MapT>, std::hash<KEY_T>, std::equal_to<KEY_T>, 
    Eigen::aligned_allocator<std::pair<const KEY_T, Submap<MapT>>>>;

} // namespace planning
} // namespace se

#endif // SE_PLANNING_SUBMAP_HPP
