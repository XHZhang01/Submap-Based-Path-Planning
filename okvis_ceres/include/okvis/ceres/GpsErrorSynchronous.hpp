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
 * @file ceres/GpsErrorSynchronous.hpp
 * @brief Header file for the GpsErrorSynchronous class.
 * @author Simon Boche
 */

#ifndef INCLUDE_OKVIS_CERES_GPSERRORSYNCHRONOUS_HPP_
#define INCLUDE_OKVIS_CERES_GPSERRORSYNCHRONOUS_HPP_

#include <vector>
#include <memory>
#include <ceres/sized_cost_function.h>
#include <ceres/covariance.h>

#include <okvis/assert_macros.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/ErrorInterface.hpp>
#include <okvis/Parameters.hpp>

namespace okvis {
namespace ceres {


/// \brief Implements a nonlinear IMU factor.
class GpsErrorSynchronous :
    public ::ceres::SizedCostFunction<3 /* number of residuals */,
        7 /* size of second parameter (RobotPoseParameterBlock T_WS) */,
        7 /* size of first parameter (PoseParameterBlock T_GW) */>,
    public ErrorInterface {

 public:

  /// \brief The base in ceres we derive from
  typedef ::ceres::SizedCostFunction<3, 7, 7> base_t;

  /// \brief The number of residuals
  static const int kNumResiduals = 3;

  /// \brief The type of the covariance.
  typedef Eigen::Matrix<double, 3, 3> covariance_t;

  /// \brief The measurement type.
  typedef Eigen::Vector3d measurement_t;

  /// \brief The type of the information (same matrix dimension as covariance).
  typedef covariance_t information_t;

  /// \brief The type of Jacobian w.r.t. robot pose in world frame
  typedef Eigen::Matrix<double, 3, 7> jacobian0_t;

  /// \brief The type of the Jacobian w.r.t. poses GPS <-> world--
  /// \warning This is w.r.t. minimal tangential space coordinates...
  typedef Eigen::Matrix<double, 3, 7> jacobian1_t;


  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// \brief Default constructor.
  GpsErrorSynchronous();

  /// \brief Construct with measurement and information matrix
  /// @param[in] cameraId The id of the camera in the okvis::cameras::NCameraSystem.
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  /// @param[in] gpsParameters The gps parameters struct
  GpsErrorSynchronous(uint64_t cameraId, const measurement_t & measurement,
                    const covariance_t & information, const GpsParameters & gpsParameters);

  /// \brief Trivial destructor.
  virtual ~GpsErrorSynchronous()
  {
  }

  // setters
  /// \brief Set gps parameters
  /// @param[in] gpsParameters the parameters of GPS sensor
  virtual void setGpsParameters(const GpsParameters & gpsParameters){
      gpsParameters_ = gpsParameters;
  }

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  virtual void setMeasurement(const measurement_t& measurement)
  {
    measurement_ = measurement;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  virtual void setInformation(const covariance_t& information);

  /// \brief Set camera ID.
  /// @param[in] cameraId ID of the camera.
  virtual void setCameraId(uint64_t cameraId) {
    cameraId_ = cameraId;
  }

  // getters
  /// \brief Camera ID.
  uint64_t cameraId() const {
    return cameraId_;
  }

  /// \brief Get the measurement.
  /// \return The measurement vector.
  virtual const measurement_t& measurement() const
  {
    return measurement_;
  }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  virtual const covariance_t& information() const
  {
    return information_;
  }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  virtual const covariance_t& covariance() const
  {
    return covariance_;
  }

  /// \brief Get the GPS parameters.
  virtual const okvis::GpsParameters& gpsParameters() const {
    return gpsParameters_;
  }

  // error term and Jacobian implementation
  /**
   * @brief This evaluates the error term and additionally computes the Jacobians.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @return success of the evaluation.
   */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  virtual bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobiansMinimal) const;

  // added convenient check
  virtual bool VerifyJacobianNumDiff(double const* const * parameters, double** jacobian) const;

  // sizes
  /// \brief Residual dimension.
  int residualDim() const
  {
    return kNumResiduals;
  }

  /// \brief Number of parameter blocks.
  int parameterBlocks() const
  {
    return parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  /// @param[in] parameterBlockId ID of the parameter block of interest.
  /// \return The dimension.
  int parameterBlockDim(int parameterBlockId) const
  {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /// @brief Residual block type as string
  virtual std::string typeInfo() const
  {
    return "GpsErrorSynchronous";
  }

 protected:

  // the measurement
  measurement_t measurement_; ///< The (3D) measurement.

  // Gps parameters
  GpsParameters gpsParameters_; ///< The gps parameters

  // weighting related
  covariance_t information_; ///< The 3x3 information matrix.
  covariance_t squareRootInformation_; ///< The 3x3 square root information matrix.
  covariance_t covariance_; ///< The 3x3 covariance matrix.

  //camera id
  uint64_t cameraId_; ///< ID of the camera.

};

}  // namespace ceres
}  // namespace okvis

#endif // /* INCLUDE_OKVIS_CERES_GPSERRORSYNCHRONOUS_HPP_ */
