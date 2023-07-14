/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_PATH_PLANNER_IMPL_HPP
#define SE_PLANNING_PATH_PLANNER_IMPL_HPP

namespace se {
namespace planning {

template<typename KEY_T, typename MapT>
PathPlanner<KEY_T, MapT>::PathPlanner(const SubmapMap<KEY_T, MapT>& maps, const PlannerConfig& config) :
        maps_(maps),
        cc_(maps, config.robot_radius),
        robot_radius_(config.robot_radius),
        goal_threshold_(1.5f * config.robot_radius),
        solving_time_(config.solving_time),
        space_(new ompl::base::RealVectorStateSpace(State::SizeAtCompileTime))
{
    // Offsets of the 8 AABB vertices relative to the minimum corner.
    // clang-format off
    static const Eigen::Array<float, 3, 8> aabb_vertex_offset = (Eigen::Array<float, 3, 8>() <<
            0, 1, 0, 1, 0, 1, 0, 1,
            0, 0, 1, 1, 0, 0, 1, 1,
            0, 0, 0, 0, 1, 1, 1, 1
    ).finished();
    // clang-format on
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    // Update the state space boundaries.
    // TODO Create a custom state space to test samples against individual submap AABBs instead of
    // computing the AABB of the AABBs? It should speed up sampling.
    ompl::base::RealVectorBounds bounds(State::SizeAtCompileTime);
    Eigen::Array3f all_map_aabb_min_W = Eigen::Array3f::Constant(INFINITY);
    Eigen::Array3f all_map_aabb_max_W = Eigen::Array3f::Constant(-INFINITY);
    for (const auto& m : maps_) {
        // Offset the AABB inwards by the robot radius since we want the robot to be completely
        // contained in the AABB.
        const Eigen::Array3f aabb_min_K = m.second.map->aabbMin() + robot_radius_;
        const Eigen::Array3f aabb_max_K = m.second.map->aabbMax() - robot_radius_;
        const Eigen::Array3f aabb_dim = aabb_max_K - aabb_min_K;
        const Eigen::Array<float, 3, 8> corners_K =
            (aabb_vertex_offset.colwise() * aabb_dim).colwise() + aabb_min_K;
        const Eigen::Array<float, 3, 8> corners_W =
            (m.second.T_WK * corners_K.matrix().colwise().homogeneous()).template topRows<3>().array();
        all_map_aabb_min_W = all_map_aabb_min_W.min(corners_W.rowwise().minCoeff());
        all_map_aabb_max_W = all_map_aabb_max_W.max(corners_W.rowwise().maxCoeff());
    }
    bounds.setLow(0, all_map_aabb_min_W.x());
    bounds.setLow(1, all_map_aabb_min_W.y());
    bounds.setLow(2, all_map_aabb_min_W.z());
    bounds.setHigh(0, all_map_aabb_max_W.x());
    bounds.setHigh(1, all_map_aabb_max_W.y());
    bounds.setHigh(2, all_map_aabb_max_W.z());

    space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    si_ = std::make_shared<ompl::base::SpaceInformation>(space_);
    // Explicitly set a StateValidityChecker to prevent an OMPL warning from popping up. We do state
    // validity checking in the MotionValidator because we don't want to test the validity of the
    // initial state. This is because:
    // 1) Testing the endpoint of each segment is enough to test every path vertex except the first.
    // 2) We don't want to test the first path vertex because it might have ended up slightly
    //    outside the map or in occupied space due to tracking inaccuracy. In that case we still
    //    want to be able to plan and not get stuck due to the start being occupied.
    si_->setStateValidityChecker(std::make_shared<ompl::base::AllValidStateValidityChecker>(si_));
    si_->setMotionValidator(std::make_shared<MotionValidator<KEY_T, MapT>>(si_, cc_));
    si_->setup();
    planner_data_ = std::make_shared<ompl::base::PlannerData>(si_);
}



template<typename KEY_T, typename MapT>
ompl::base::PlannerStatus PathPlanner<KEY_T, MapT>::plan(const Eigen::Vector3f& start_t_WB,
                                                  const Eigen::Vector3f& goal_t_WB)
{
    // Ensure the start point is inside the map even if tracking errors made the robot end up a
    // little outside the map. Project the start point inside all submaps that don't contain it.
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> start_t_WB_safe_vec;
    for (const auto& m : maps_) {
        const Eigen::Vector3f start_t_KB = (m.second.T_KW * start_t_WB.homogeneous()).template head<3>();
        if (m.second.map->contains(start_t_KB)) {
            continue;
        }
        // Offset the map AABB inwards by the robot radius so that the safe start is completely
        // inside the map.
        const Eigen::Vector3f map_aabb_min = (m.second.map->aabbMin() + (robot_radius_ + 1e-5)).matrix();
        const Eigen::Vector3f map_aabb_max = (m.second.map->aabbMax() - (robot_radius_ + 1e-5)).matrix();
        const Eigen::Vector3f start_t_KB_safe =
            start_t_KB.cwiseMax(map_aabb_min).cwiseMin(map_aabb_max);
        start_t_WB_safe_vec.push_back((m.second.T_WK * start_t_KB_safe.homogeneous()).template head<3>());
    }
    const bool start_inside = start_t_WB_safe_vec.size() < maps_.size();
    const Eigen::Vector3f start_t_WB_safe = start_inside
        ? start_t_WB
        : *std::min_element(start_t_WB_safe_vec.begin(),
                            start_t_WB_safe_vec.end(),
                            [&](const auto& lhs, const auto& rhs) {
                                return (start_t_WB - lhs).squaredNorm()
                                    < (start_t_WB - rhs).squaredNorm();
                            });

    if (start_t_WB_safe.isApprox(goal_t_WB)) {
        path_.push_back(start_t_WB_safe);
        path_.push_back(goal_t_WB);
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::EXACT_SOLUTION);
    }

    std::cout << "Start is " << start_t_WB_safe.transpose() << std::endl;
    std::cout << "End is " << goal_t_WB.transpose() << std::endl;
    setupPlanner(start_t_WB_safe, goal_t_WB);
    const ompl::base::PlannerStatus status = planner_->solve(solving_time_);
    if (!status) {
        return status;
    }
    // Simplify the path and convert to the appropriate type.
    ompl::geometric::PathGeometric& path =
        *pdef_->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    simplifyPath(path);
    path_ = convert_path(path);
    if (!start_inside) {
        path_.emplace(path_.begin(), start_t_WB);
    }
    // TODO Consider checking that the approximate solution is within some threshold of the goal and
    // return failed otherwise.
    return status;
}



template<typename KEY_T, typename MapT>
const Path& PathPlanner<KEY_T, MapT>::path() const
{
    return path_;
}



template<typename KEY_T, typename MapT>
const ompl::base::PlannerData& PathPlanner<KEY_T, MapT>::plannerData() const
{
    // getPlannerData() will append to planner_data_ but we're using a one-shot planner so old
    // PlannerData is useless so it must be cleared first.
    planner_data_->clear();
    planner_->getPlannerData(*planner_data_);
    return *planner_data_;
}



template<typename KEY_T, typename MapT>
int PathPlanner<KEY_T, MapT>::savePath(const std::string& filename) const
{
    std::ofstream f(filename);
    if (!f.good()) {
        return 1;
    }
    f << "$fa = 1;\n$fs = 0.05;\n\n";
    f << "module Position(position, radius) {\n";
    f << "\ttranslate(position) sphere(radius);\n";
    f << "}\n\n";
    f << "module Segment(start, end, radius) {\n";
    f << "\tcentre = (start + end) / 2;\n";
    f << "\theight = norm(end - start);\n";
    f << "\tdir = (end - start) / height;\n";
    f << "\tangle = acos([0, 0, 1] * dir);\n";
    f << "\taxis = cross([0, 0, 1], dir);\n";
    f << "\ttranslate(centre) rotate(a=angle, v=axis) cylinder(h=height, r=radius, center=true);\n";
    f << "}\n\n";
    for (size_t i = 0; i < path_.size(); i++) {
        const auto& v = path_[i];
        f << "p" << i << " = [" << v.x() << ", " << v.y() << ", " << v.z() << "];\n";
    }
    f << "r = " << robot_radius_ << ";\n\n";
    for (size_t i = 0; i < path_.size(); i++) {
        f << "Position(p" << i << ", r);\n";
        if (i + 1 < path_.size()) {
            f << "Segment(p" << i << ", p" << i + 1 << ", r);\n";
        }
    }
    return 0;
}



template<typename KEY_T, typename MapT>
void PathPlanner<KEY_T, MapT>::setupPlanner(const Eigen::Vector3f& start_t_WB,
                                     const Eigen::Vector3f& goal_t_WB)
{
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> ompl_start(space_), ompl_goal(space_);
    eigen_to_ompl(start_t_WB, *ompl_start);
    eigen_to_ompl(goal_t_WB, *ompl_goal);
    pdef_ = std::shared_ptr<ompl::base::ProblemDefinition>(new ompl::base::ProblemDefinition(si_));
    pdef_->setStartAndGoalStates(ompl_start, ompl_goal, goal_threshold_);

    // Optimise for path length. Set a high cost (path length) threshold so that planning
    // practically stops on the first path found.
    ompl::base::OptimizationObjectivePtr objective(
        new ompl::base::PathLengthOptimizationObjective(si_));
    objective->setCostThreshold(ompl::base::Cost(10000));
    pdef_->setOptimizationObjective(objective);

    planner_ = std::shared_ptr<ompl::geometric::InformedRRTstar>(
        new ompl::geometric::InformedRRTstar(si_));
    planner_->setProblemDefinition(pdef_);
    planner_->setup();
}



template<typename KEY_T, typename MapT>
void PathPlanner<KEY_T, MapT>::simplifyPath(ompl::geometric::PathGeometric& path)
{
    if (path.getStateCount() < 3) {
        return;
    }
    ompl::geometric::PathSimplifier simplifier(si_);
    // Termination condition
    const float max_time = 0.5; // TODO: parameterize
    ompl::base::PlannerTerminationCondition stop =
        ompl::base::timedPlannerTerminationCondition(max_time);
    // Try a randomized step of connecting vertices.
    bool try_more = stop ? false : simplifier.reduceVertices(path);
    // Try to collapse close-by vertices.
    if (!stop) {
        simplifier.collapseCloseVertices(path);
    }
    // Try to reduce vertices some more, if there is any point in doing so.
    int times = 0;
    while (try_more && !stop && ++times <= 5) {
        try_more = simplifier.reduceVertices(path);
    }
}

} // namespace planning
} // namespace se

#endif // SE_PLANNING_PATH_PLANNER_IMPL_HPP
