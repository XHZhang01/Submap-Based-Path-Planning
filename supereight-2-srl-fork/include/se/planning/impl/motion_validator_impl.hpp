/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_MOTION_VALIDATOR_IMPL_HPP
#define SE_PLANNING_MOTION_VALIDATOR_IMPL_HPP

namespace se {
namespace planning {

template<typename KEY_T, typename MapT>
MotionValidator<KEY_T, MapT>::MotionValidator(const ompl::base::SpaceInformationPtr& si,
                                       const CollisionChecker<KEY_T, MapT>& cc) :
        ompl::base::MotionValidator(si),
        cc_(cc),
        state_space_(si_ ? si_->getStateSpace().get() : nullptr)
{
    if (!state_space_) {
        throw std::runtime_error("No state space in se::planning::MotionValidator");
    }
}



template<typename KEY_T, typename MapT>
bool MotionValidator<KEY_T, MapT>::checkMotion(const ompl::base::State* s1,
                                        const ompl::base::State* s2) const
{
    if (!si_->satisfiesBounds(s2) || !cc_.isSegmentFree(ompl_to_eigen(s1), ompl_to_eigen(s2))) {
        invalid_++;
        return false;
    }
    valid_++;
    return true;
}



template<typename KEY_T, typename MapT>
bool MotionValidator<KEY_T, MapT>::checkMotion(const ompl::base::State* s1,
                                        const ompl::base::State* s2,
                                        std::pair<ompl::base::State*, double>& last_valid) const
{
    if (!si_->satisfiesBounds(s2)) {
        if (last_valid.first) {
            last_valid.second = 0.0;
            state_space_->interpolate(s1, s2, last_valid.second, last_valid.first);
        }
        invalid_++;
        return false;
    }
    ompl::base::State* curr = si_->allocState();
    ompl::base::State* prev = si_->allocState();
    const int nd = state_space_->validSegmentCount(s1, s2);
    for (int j = 1; j <= nd; ++j) {
        state_space_->interpolate(s1, s2, static_cast<double>(j) / nd, curr);
        state_space_->interpolate(s1, s2, static_cast<double>(j - 1) / nd, prev);
        if (!cc_.isSegmentFree(ompl_to_eigen(prev), ompl_to_eigen(curr))) {
            if (last_valid.first) {
                last_valid.second = static_cast<double>(j - 1) / nd;
                state_space_->interpolate(s1, s2, last_valid.second, last_valid.first);
            }
            si_->freeState(curr);
            si_->freeState(prev);
            invalid_++;
            return false;
        }
    }
    si_->freeState(curr);
    si_->freeState(prev);
    valid_++;
    return true;
}

} // namespace planning
} // namespace se

#endif // SE_PLANNING_MOTION_VALIDATOR_IMPL_HPP
