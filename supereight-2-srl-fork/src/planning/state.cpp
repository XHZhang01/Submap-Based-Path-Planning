/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/planning/state.hpp"

namespace se {
namespace planning {

State ompl_to_eigen(const ompl::base::State* ompl_state)
{
    const auto& state = *ompl_state->as<ompl::base::RealVectorStateSpace::StateType>();
    return State(state[0], state[1], state[2]);
}



void eigen_to_ompl(const State& state, ompl::base::RealVectorStateSpace::StateType& ompl_state)
{
    ompl_state[0] = state.x();
    ompl_state[1] = state.y();
    ompl_state[2] = state.z();
}



Path convert_path(const ompl::geometric::PathGeometric& ompl_path)
{
    Path path;
    for (size_t i = 0; i < ompl_path.getStateCount(); i++) {
        const auto& state =
            *ompl_path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        path.emplace_back(state[0], state[1], state[2]);
    }
    return path;
}

PoseVector path_to_poses(const Path& path, const Eigen::Quaterniond& q_WB, const Eigen::Quaterniond& q_WBg)
{
    PoseVector poses(path.size(), Eigen::Matrix4f::Identity());
    for (size_t i = 0; i < poses.size(); i++) {
        poses[i].topRightCorner<3,1>() = path[i];
        // The orientation of intermediate path poses is interpolated
        // between the current and goal orienations.
        const double a = static_cast<double>(i) / (poses.size() - 1);
        poses[i].topLeftCorner<3,3>() = (q_WB.slerp(a, q_WBg).toRotationMatrix()).template cast<float>();
    }

    return poses;
}

PoseVector pose_oversampling(const PoseVector& path, const float oversampling_dist)
{   
    
    PoseVector final_path;
    if(path.empty()) {
        return final_path;
    }
    
    for(size_t i = 1; i < path.size(); i++){
        Eigen::Vector3f start = path[i - 1].topRightCorner<3, 1>();
        Eigen::Quaternionf start_ori(path[i - 1].topLeftCorner<3, 3>());
        Eigen::Vector3f end = path[i].topRightCorner<3, 1>();
        Eigen::Quaternionf end_ori(path[i].topLeftCorner<3, 3>());
        
        Eigen::Vector3f udir = (end - start);
        int samples = std::ceil(udir.norm() / oversampling_dist);
        int counter = 1;
        udir.normalize();
        
        
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        final_path.push_back(path[i - 1]);
        for(counter; counter < samples; counter++){
            Eigen::Vector3f temp_pos = start + counter * oversampling_dist * udir;
            Eigen::Quaternionf temp_ori = start_ori.slerp(static_cast<float>(counter) / samples, end_ori);

            pose.topRightCorner<3, 1>() = temp_pos;
            pose.topLeftCorner<3, 3>() = temp_ori.toRotationMatrix();
            final_path.push_back(pose);

        }

    }

    final_path.push_back(path[path.size() - 1]);
    return final_path;
}
} // namespace planning
} // namespace se
