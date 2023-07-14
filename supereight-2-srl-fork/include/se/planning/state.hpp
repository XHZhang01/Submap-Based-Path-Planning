/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_STATE_HPP
#define SE_PLANNING_STATE_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <vector>

namespace se {
namespace planning {

typedef Eigen::Vector3f State;

typedef std::vector<State, Eigen::aligned_allocator<State>> Path;

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> PoseVector;

State ompl_to_eigen(const ompl::base::State* ompl_state);

void eigen_to_ompl(const State& state, ompl::base::RealVectorStateSpace::StateType& ompl_state);

Path convert_path(const ompl::geometric::PathGeometric& ompl_path);

/**
* @brief Given a path composed of 3D positions, it converts it into a vector of poses. The interpolated angle is done from start to end
* @param[in] path The path to be converted
* @param[in] q_WB The quaternion of the start pose
* @param[in] q_WBg The quaternion of the goal pose
* @return A vector of poses of the path
*/
PoseVector path_to_poses(const Path& path, const Eigen::Quaterniond& q_WB, const Eigen::Quaterniond& q_WBg);

/**
* @brief Given a trajectory, it samples more poses to follow between the trajectory points
* @param[in] path The path to be sampled
* @return A path with more points to follow
*/
PoseVector pose_oversampling(const PoseVector& path, const float oversampling_dist);

} // namespace planning
} // namespace se

#endif // SE_PLANNING_STATE_HPP
