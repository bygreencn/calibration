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


#ifndef PROJECTION_H
#define PROJECTION_H

#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>

namespace calib
{

/// \brief Project a 3D point into a 2D point using the camera model (cam_model)
void projectPoints(const image_geometry::PinholeCameraModel &cam_model,
                   const cv::Point3d &points3D,
                   cv::Point2d *points2D);

/// \brief Project 3D points into 2D points using the camera model (cam_model)
void projectPoints(const image_geometry::PinholeCameraModel &cam_model,
                   const std::vector<cv::Point3d> &points3D,
                   std::vector<cv::Point2d> *points2D);

/// \brief Calculate reprojection error
double computeReprojectionErrors(cv::InputArray points3D,
                                 cv::InputArray points2D,
                                 cv::InputArray cameraMatrix,
                                 cv::InputArray distCoeffs,
                                 cv::InputArray rvec,
                                 cv::InputArray tvec,
                                 cv::OutputArray proj_points2D =cv::noArray());

/// \brief Calculate reprojection error
double computeReprojectionErrors(double points3D[3],
                                 double points2D[2],
                                 double fx, double fy, // intrinsics
                                 double cx, double cy, // intrinsics
                                 double camera_rotation[4],
                                 double camera_translation[3],
                                 double proj_points2D[2] =0);


/// \brief Transform 3D points using rvec (3x1 or 1x3 or 3x3) and tvec (3x1)
void transform3DPoints(const cv::Mat &points,
                       const cv::Mat &rvec,
                       const cv::Mat &tvec,
                       cv::Mat *modif_points );

}

#endif // PROJECTION_H

