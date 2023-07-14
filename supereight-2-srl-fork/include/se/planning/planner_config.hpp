/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_PLANNER_CONFIG_HPP
#define SE_PLANNING_PLANNER_CONFIG_HPP

#include <Eigen/Dense>

namespace se {
namespace planning {

struct PlannerConfig {
    /** \brief The radius of the robot's bounding sphere in metres. */
    float robot_radius = 0.15f;

    /** \brief The number of seconds a path will be searched for. */
    float solving_time = 0.1f;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace planning
} // namespace se

#endif // SE_PLANNING_PLANNER_CONFIG_HPP
