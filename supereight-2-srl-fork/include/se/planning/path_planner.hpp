/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_PATH_PLANNER_HPP
#define SE_PLANNING_PATH_PLANNER_HPP

#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include "collision_checker.hpp"
#include "motion_validator.hpp"
#include "planner_config.hpp"
#include "se/common/math_util.hpp"
#include "se/map/map.hpp"
#include "state.hpp"

namespace se {
namespace planning {

template<typename KEY_T, typename MapT>
class PathPlanner {
    public:
    PathPlanner(const SubmapMap<KEY_T, MapT>& maps, const PlannerConfig& config);

    /** \brief Plan a path from a start to a goal position.
     * The start and goal are the position of the body frame B expressed in the world frame W.
     */
    ompl::base::PlannerStatus plan(const Eigen::Vector3f& start_t_WB,
                                   const Eigen::Vector3f& goal_t_WB);

    const Path& path() const;

    const ompl::base::PlannerData& plannerData() const;

    /** \brief Save the path as an OpenSCAD script in filename.
     * OpenSCAD has sphere and cylinder primitives so we avoid having to do sphere and cylinder
     * meshing.
     */
    int savePath(const std::string& filename) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    void setupPlanner(const Eigen::Vector3f& start_t_WB, const Eigen::Vector3f& goal_t_WB);
    void simplifyPath(ompl::geometric::PathGeometric& path);

    const SubmapMap<KEY_T, MapT> maps_;
    const CollisionChecker<KEY_T, MapT> cc_;
    const float robot_radius_;
    const float goal_threshold_;
    const float solving_time_;

    ompl::base::StateSpacePtr space_;
    ompl::base::SpaceInformationPtr si_;
    ompl::base::ProblemDefinitionPtr pdef_;
    ompl::base::PlannerPtr planner_;
    mutable ompl::base::PlannerDataPtr planner_data_;

    Path path_;
};

} // namespace planning
} // namespace se

#include "impl/path_planner_impl.hpp"

#endif // SE_PLANNING_PATH_PLANNER_HPP
