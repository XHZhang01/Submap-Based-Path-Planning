/*
 * SPDX-FileCopyrightText: 2018 ETH Zürich
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018-2020 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PLANNING_MOTION_VALIDATOR_HPP
#define SE_PLANNING_MOTION_VALIDATOR_HPP

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>

#include "collision_checker.hpp"

namespace se {
namespace planning {

/** \brief MotionValidator checks the validity of segments between states.
 * It assumes the start state is valid.
 */
template<typename KEY_T, typename MapT>
class MotionValidator : public ompl::base::MotionValidator {
    public:
    MotionValidator(const ompl::base::SpaceInformationPtr& si, const CollisionChecker<KEY_T, MapT>& cc);

    bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

    bool checkMotion(const ompl::base::State* s1,
                     const ompl::base::State* s2,
                     std::pair<ompl::base::State*, double>& last_valid) const override;

    private:
    const CollisionChecker<KEY_T, MapT>& cc_;
    const ompl::base::StateSpace* state_space_;
};

} // namespace planning
} // namespace se

#include "impl/motion_validator_impl.hpp"

#endif // SE_PLANNING_MOTION_VALIDATOR_HPP
