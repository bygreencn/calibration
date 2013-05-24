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

#include "conversion.h"
#include "auxiliar.h"

using namespace cv;
using namespace std;

namespace calib
{

void projectPoints(const image_geometry::PinholeCameraModel &cam_model,
                   const cv::Point3d &points3D,
                   cv::Point2d *points2D)
{
  *points2D = cam_model.project3dToPixel(points3D);
//   *points2D = cam_model.rectifyPoint(*points2D);
}

void projectPoints(const image_geometry::PinholeCameraModel &cam_model,
                   const vector<cv::Point3d> &xyz,
                   vector<cv::Point2d> *points2D)
{
  size_t n = xyz.size();
  points2D->clear();
  points2D->reserve(n);

  for (size_t i = 0; i < n; i++)
  {
    cv::Point2d current_pt;
    projectPoints(cam_model, xyz[i], &current_pt);
    points2D->push_back(current_pt);
  }
}

double computeReprojectionErrors(InputArray points3D,
                                 InputArray points2D,
                                 InputArray cameraMatrix,
                                 InputArray distCoeffs,
                                 InputArray rvec,
                                 InputArray tvec,
                                 OutputArray _proj_points2D,
                                 vector<double> *individual_error)
{
  // set proper type for the output
  Mat x = points2D.getMat();
  Mat proj_points2D = _proj_points2D.getMat();
  proj_points2D.create(x.rows, x.cols, x.type());

  // project points
  projectPoints(points3D, rvec, tvec, cameraMatrix, distCoeffs, proj_points2D);

  // save output if it is needed (no default parameter)
  if (_proj_points2D.needed())
  {
    proj_points2D.copyTo(_proj_points2D);
  }

  // return error
  return calib::norm(x, proj_points2D, individual_error);
}

void transform3DPoints(const Mat &points,
                       const Mat &rvec,
                       const Mat &tvec,
                       Mat *modif_points)
{
  Mat transformation;

  if ((rvec.rows == 3 && rvec.cols == 1) || (rvec.rows == 1 && rvec.cols == 3))
  {
    Mat R;
    Rodrigues(rvec, R);

    cv::hconcat(R, tvec, transformation);
  }
  else
    if (rvec.rows == 3 && rvec.cols == 3)
    {
      cv::hconcat(rvec, tvec, transformation);
    }
    else
    {
      cerr << "wrong size!" << endl;
      return;
    }

  transform(points, *modif_points, transformation);
}

void transform3DPoints(const cv::Mat &points,
                       const KDL::Frame &frame,
                       cv::Mat *modif_points)
{
  cv::Mat R, t;
  kdl2cv(frame, R, t);

  Mat transformation;
  cv::hconcat(R, t, transformation);

  transform(points, *modif_points, transformation);
}

}
