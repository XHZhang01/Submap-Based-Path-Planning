/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Aug 30, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file PoseLocalParameterization.hpp
 * @brief Header file for the PoseLocalParemerization class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_POSELOCALPARAMETERIZATION_HPP_
#define INCLUDE_OKVIS_CERES_POSELOCALPARAMETERIZATION_HPP_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <ceres/manifold.h>
#pragma GCC diagnostic pop

#include <okvis/assert_macros.hpp>
#include <okvis/kinematics/operators.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// \brief A simple pose manifold parameterisation using ambient space R^3 x SO(3).
class PoseManifold : public ::ceres::Manifold {
 public:
  /// \brief Default destructor.
  virtual ~PoseManifold() override = default;

  /// \brief Dimension of the ambient space (R^3 x SO(3)) in which the manifold is embedded.
  /// \return  The dimension (7).
  virtual int AmbientSize() const override {return 7;}

  /// \brief Dimension of the tangent space (R^6).
  /// \return  The dimension (6).
  virtual int TangentSize() const override {return 6;}

  /// \brief Generalised addition: x_plus_delta = Plus(x, delta).
  /// @param[in] x x.
  /// @param[in] delta delta.
  /// @param[out] x_plus_delta x_plus_delta.
  /// \return true on success.
  static bool plus(const double* x,
                   const double* delta,
                   double* x_plus_delta);

  /// \brief Generalised addition: x_plus_delta = Plus(x, delta).
  /// @param[in] x x.
  /// @param[in] delta delta.
  /// @param[out] x_plus_delta x_plus_delta.
  /// \return true on success.
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const override;

  /// \brief Jacobian of the plus operation (generalised addition: x_plus_delta = Plus(x, delta)).
  /// @param[in] x x.
  /// @param[out] jacobian The jacobian.
  /// \return true on success.
  static bool plusJacobian(const double* x, double* jacobian);

  /// \brief Jacobian of the plus operation (generalised addition: x_plus_delta = Plus(x, delta)).
  /// @param[in] x x.
  /// @param[out] jacobian The jacobian.
  /// \return true on success.
  virtual bool PlusJacobian(const double* x, double* jacobian) const override;

  /// \brief Generalised subtraction: y_minus_x = Minus(y, x).
  /// @param[in] y y.
  /// @param[in] x x.
  /// @param[out] y_minus_x y_minus_x.
  /// \return true on success.
  static bool minus(const double* y,
                    const double* x,
                    double* y_minus_x);

  /// \brief Generalised subtraction: y_minus_x = Minus(y, x).
  /// @param[in] y y.
  /// @param[in] x x.
  /// @param[out] y_minus_x y_minus_x.
  /// \return true on success.
  virtual bool Minus(const double* y,
                     const double* x,
                     double* y_minus_x) const override;

  /// \brief Jacobian of the minus operation (generalised subtraction: y_minus_x = Minus(y, x)).
  /// @param[in] x x.
  /// @param[out] jacobian The Jacobian.
  /// \return true on success.
  static bool minusJacobian(const double* x, double* jacobian);

  /// \brief Jacobian of the minus operation (generalised subtraction: y_minus_x = Minus(y, x)).
  /// @param[in] x x.
  /// @param[out] jacobian The Jacobian.
  /// \return true on success.
  virtual bool MinusJacobian(const double* x, double* jacobian) const override;

  /// \brief Plus jacobian verification using numeric differences for convenience.
  /// @param[in] x x.
  /// @param[out] jacobian The Jacobian.
  /// @param[out] jacobianNumDiff The Jacobian with numeric differences.
  /// \return true on success.
  bool verifyJacobianNumDiff(const double* x, double* jacobian,
                        double* jacobianNumDiff) const;
};

/// \brief A simple pose manifold parameterisation for 4DoF transformation.
/// Here, we only perturb the translation and yaw though.
class PoseManifold4d : public ::ceres::Manifold {
 public:
  /// \brief Default destructor.
  virtual ~PoseManifold4d() override = default;

  /// \brief Dimension of the ambient space (R^3 x SO(3)) in which the manifold is embedded.
  /// \return  The dimension (7).
  virtual int AmbientSize() const override {return 7;}

  /// \brief Dimension of the tangent space (R^6).
  /// \return  The dimension (4).
  virtual int TangentSize() const override {return 4;}

  /// \brief Generalised addition: x_plus_delta = Plus(x, delta).
  /// @param[in] x x.
  /// @param[in] delta delta.
  /// @param[out] x_plus_delta x_plus_delta.
  /// \return true on success.
  static bool plus(const double* x,
                   const double* delta,
                   double* x_plus_delta);

  /// \brief Generalised addition: x_plus_delta = Plus(x, delta).
  /// @param[in] x x.
  /// @param[in] delta delta.
  /// @param[out] x_plus_delta x_plus_delta.
  /// \return true on success.
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const override;

  /// \brief Jacobian of the plus operation (generalised addition: x_plus_delta = Plus(x, delta)).
  /// @param[in] x x.
  /// @param[out] jacobian The jacobian.
  /// \return true on success.
  static bool plusJacobian(const double* x, double* jacobian);

  /// \brief Jacobian of the plus operation (generalised addition: x_plus_delta = Plus(x, delta)).
  /// @param[in] x x.
  /// @param[out] jacobian The jacobian.
  /// \return true on success.
  virtual bool PlusJacobian(const double* x, double* jacobian) const override;

  /// \brief Generalised subtraction: y_minus_x = Minus(y, x).
  /// @param[in] y y.
  /// @param[in] x x.
  /// @param[out] y_minus_x y_minus_x.
  /// \return true on success.
  static bool minus(const double* y,
                    const double* x,
                    double* y_minus_x);

  /// \brief Generalised subtraction: y_minus_x = Minus(y, x).
  /// @param[in] y y.
  /// @param[in] x x.
  /// @param[out] y_minus_x y_minus_x.
  /// \return true on success.
  virtual bool Minus(const double* y,
                     const double* x,
                     double* y_minus_x) const override;

  /// \brief Jacobian of the minus operation (generalised subtraction: y_minus_x = Minus(y, x)).
  /// @param[in] x x.
  /// @param[out] jacobian The Jacobian.
  /// \return true on success.
  static bool minusJacobian(const double* x, double* jacobian);

  /// \brief Jacobian of the minus operation (generalised subtraction: y_minus_x = Minus(y, x)).
  /// @param[in] x x.
  /// @param[out] jacobian The Jacobian.
  /// \return true on success.
  virtual bool MinusJacobian(const double* x, double* jacobian) const override;


};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_POSELOCALPARAMETERIZATION_HPP_ */
