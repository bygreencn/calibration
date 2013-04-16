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

#include "projection.h"

#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

namespace calib
{

void projectPoints(const image_geometry::PinholeCameraModel &cam_model,
                   const cv::Point3d &xyz,
                   cv::Point2d *points2D)
{
  *points2D = cam_model.project3dToPixel(xyz);
//   *points2D = cam_model.rectifyPoint(*points2D);
}

void projectPoints(const image_geometry::PinholeCameraModel &cam_model,
                   const vector<cv::Point3d> &xyz,
                   vector<cv::Point2d> *points2D)
{
  size_t n = xyz.size();
  points2D->clear();
  points2D->reserve(n);

  for(size_t i=0; i<n; i++)
  {
    cv::Point2d current_pt;
    projectPoints(cam_model, xyz[i], &current_pt);
    points2D->push_back(current_pt);
  }
}

double computeReprojectionErrors(const vector<Point3d> &X3D,
                                 const vector<Point2d> &x2d,
                                 const Mat &cameraMatrix,
                                 const Mat &distCoeffs,
                                 const Mat &rvec,
                                 const Mat &tvec)
{
  double err;
  vector<Point2d> x2d_proj;

  projectPoints(X3D, rvec, tvec, cameraMatrix, distCoeffs, x2d_proj);
  err = cv::norm(x2d, x2d_proj, CV_L2);

//   cout << "\t x2d = " << x2d << endl;
//   cout << "\t x2d_proj ="  << x2d_proj << endl << endl;

  return err;
}

void project3dPoints(const Mat &points,
                     const Mat &rvec,
                     const Mat &tvec,
                     Mat *modif_points )
{
  modif_points->create(1, points.cols, CV_32FC3);
  Mat R(3, 3, CV_64FC1);
  Rodrigues(rvec, R);
  Mat transformation(3, 4, CV_64F);
  Mat r = transformation.colRange(0, 3);
  R.copyTo(r);
  Mat t = transformation.colRange(3, 4);
  tvec.copyTo(t);
  transform(points, *modif_points, transformation);
}


}
