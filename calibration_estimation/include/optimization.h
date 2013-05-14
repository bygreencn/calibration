/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//! \author Pablo Speciale


#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <opencv2/core/mat.hpp>

#include "projection.h"

namespace calib
{

  struct ReprojectionErrorWithQuaternions {
  ReprojectionErrorWithQuaternions(double observed_x, double observed_y,
                                   double fx, double fy, double cx, double cy)
    : observed_x(observed_x), observed_y(observed_y),
      fx(fx), fy(fy), cx(cx), cy(cy) {}

  template <typename T>
  bool operator()(const T* const camera_rotation,
                  const T* const camera_translation,
                  const T* const point,
                  T *residuals) const
  {
    // camera_rotation are the quaternions
    T p[3];
    ceres::QuaternionRotatePoint(camera_rotation, point, p);

    // camera_translation is the translation
    p[0] += camera_translation[0];
    p[1] += camera_translation[1];
    p[2] += camera_translation[2];

    // Compute the projection
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    T predicted_x = T(fx) * xp + T(cx);
    T predicted_y = T(fy) * yp + T(cy);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double fx,
                                     const double fy,
                                     const double cx,
                                     const double cy) {
    return (new ceres::AutoDiffCostFunction<ReprojectionErrorWithQuaternions, 2, 4, 3, 3>(
                new ReprojectionErrorWithQuaternions(observed_x, observed_y, fx, fy, cx, cy)));
  }

  double observed_x;
  double observed_y;
  double fx;
  double fy;
  double cx;
  double cy;
};


struct ReprojectionErrorWithQuaternions2 {
  ReprojectionErrorWithQuaternions2(double observed_x, double observed_y,
                                   double fx, double fy, double cx, double cy, double *_point)
    : observed_x(observed_x), observed_y(observed_y),
      fx(fx), fy(fy), cx(cx), cy(cy) {
        point[0] = _point[0];
        point[1] = _point[1];
        point[2] = _point[2];
      }

//   template <typename double>
  bool operator()(const double* camera_rotation,
                  const double* camera_translation,
                  double *residuals) const
  {
//     double p[2];
//     double proj_p[2];
//
//     p[0] = observed_x;
//     p[1] = observed_y;
//     double current_error = computeReprojectionErrors(point, p,
//                                            fx, fy, cx, cy,
//                                            camera_rotation,
//                                            camera_translation,
//                                            proj_p);
//
//     residuals[0] = proj_p[0] - double(observed_x);
//     residuals[1] = proj_p[1] - double(observed_y);

    // camera_rotation are the quaternions
    double p[3];
    ceres::QuaternionRotatePoint(camera_rotation, point, p);

    // camera_translation is the translation
    p[0] += camera_translation[0];
    p[1] += camera_translation[1];
    p[2] += camera_translation[2];

    // Compute the projection
    double xp = p[0] / p[2];
    double yp = p[1] / p[2];
    double predicted_x = double(fx) * xp + double(cx);
    double predicted_y = double(fy) * yp + double(cy);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - double(observed_x);
    residuals[1] = predicted_y - double(observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double fx,
                                     const double fy,
                                     const double cx,
                                     const double cy,
                                     double *point) {
    return (new ceres::NumericDiffCostFunction<ReprojectionErrorWithQuaternions2, ceres::CENTRAL, 2, 4, 3>(
                new ReprojectionErrorWithQuaternions2(observed_x, observed_y, fx, fy, cx, cy, point)));
  }

  double observed_x;
  double observed_y;
  double fx;
  double fy;
  double cx;
  double cy;
  double point[3];
};



}

#endif // OPTIMIZATION_H


