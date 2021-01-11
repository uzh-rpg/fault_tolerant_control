/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
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
 *  Created on: Sep 12, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Zurich Eye
 *********************************************************************************/

/**
 * @file Landmark_location_error.cpp
 * @brief Source file for the Landmark_location_error class.
 * @author Stefan Leutenegger
 */

#include <ze/common/transformation.hpp>
#include <ze/nlls/estimator_types.hpp>
#include <ze/nlls/landmark_location_error.hpp>
#include <ze/nlls/pose_local_parameterization.hpp>

namespace ze {
namespace nlls {

// Construct with measurement and information matrix.
LandmarkLocationError::LandmarkLocationError(
    const information_t& information)
{
  setInformation(information);
}

LandmarkLocationError::LandmarkLocationError(const information_t & information, const double & height_mea)
{
  setInformation(information);
  setHeight(height_mea);
}


// Set the information.
void LandmarkLocationError::setInformation(const information_t& information)
{
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  // square_root_information_ = sqrt(information);
  square_root_information_ = information;
}

void LandmarkLocationError::setHeight(const double& height)
{
  height_ = height;  
}

// This evaluates the error term and additionally computes the Jacobians.
bool LandmarkLocationError::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool LandmarkLocationError::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
    // std::cout << "Im here!" << std::endl;

    // double error = parameters[0][2] - (-0.7); // -0.7 is the height of ground 
    
    double error_distance = (parameters[1][2] - parameters[0][2] - height_); // || Z - Pz - h ||
    double error_location = (parameters[0][2] + 1.0); // ||Pz - 0.0|| enforce all landmarks on the ground

    Eigen::Map<Eigen::Matrix<double, 2, 1> > weighted_error(residuals);
    weighted_error(0) = square_root_information_(0,0) * error_distance;
    weighted_error(1) = square_root_information_(1,1) * error_location;

  if (jacobians != nullptr)
    {
      if (jacobians[0] != nullptr)
      {

        // compute the minimal version
        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J0_minimal;

        J0_minimal << 0.0, 0.0, -square_root_information_(0,0),
                      0.0, 0.0,  square_root_information_(1,1);

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor> >
            J0(jacobians[0]);
        J0 << J0_minimal, Eigen::MatrixXd::Zero(2,1);
  
        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[0] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >
                J0_minimal_mapped(jacobians_minimal[0]);
            J0_minimal_mapped = J0_minimal;
          }
        }
      }
      if (jacobians[1] != nullptr)
      {

        // compute the minimal version
        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J1_minimal;

        J1_minimal << 0.0, 0.0, square_root_information_(0,0), 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> >
            J1(jacobians[1]);
        J1 << J1_minimal, Eigen::MatrixXd::Zero(2,1);
  
        // if requested, provide minimal Jacobians
        if (jacobians_minimal != nullptr)
        {
          if (jacobians_minimal[1] != nullptr)
          {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
                J1_minimal_mapped(jacobians_minimal[1]);
            J1_minimal_mapped = J1_minimal;
          }
        }
      }

    }


  return true;

}

}  // namespace nlls
}  // namespace ze
