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

#include "conversion.h"
#include <ros/assert.h>

using namespace std;
using namespace cv;

namespace calib
{

void ros2cv(const geometry_msgs::Point &point_ros, Point3d *point_cv)
{
  point_cv->x = point_ros.x;
  point_cv->y = point_ros.y;
  point_cv->z = point_ros.z;
}

void ros2cv(const vector<geometry_msgs::Point> &pts_ros, vector<Point3d> *pts_cv)
{
  pts_cv->clear();
  pts_cv->reserve(pts_ros.size());

  vector<geometry_msgs::Point>::const_iterator it = pts_ros.begin();
  for(; it < pts_ros.end(); ++it)
  {
    Point3d current_pt;
    ros2cv(*it,&current_pt);
    pts_cv->push_back(current_pt);
  }
}

void ros2cv(const vector<geometry_msgs::Point> &pts_ros, Mat_<double> *pts_cv)
{
  vector<Point3d> points;
  ros2cv(pts_ros, &points);
  *pts_cv = Mat(points);
  transpose(*pts_cv, *pts_cv);
}

void cv2ros(const Mat &pts_cv, vector<geometry_msgs::Point> *pts_ros)
{
  ROS_ASSERT(pts_cv.type()==CV_64FC3 && pts_cv.cols==1);

  // Mat -> vector<Point3d>
  vector<Point3d> v_point;
  v_point.clear();
  v_point = (vector<Point3d>) pts_cv;

  // vector<Point3d> -> vector<geometry_msgs::Point>
  pts_ros->clear();
  pts_ros->reserve(v_point.size());
  for( unsigned int i = 0; i < v_point.size(); i++ )
  {
    geometry_msgs::Point current;
    current.x = v_point[i].x;
    current.y = v_point[i].y;
    current.z = v_point[i].z;

    pts_ros->push_back(current);
  }
}

void kdl2cv(const KDL::Frame &frame,
            OutputArray _R,
            OutputArray _t)
{
  kdl2cv(frame.M, _R);
  kdl2cv(frame.p, _t);
}

void kdl2cv(const KDL::Rotation &rotation, cv::OutputArray _R)
{
  // get rotation
  cv::Matx33d R;

  R(0, 0) = rotation(0, 0);
  R(0, 1) = rotation(0, 1);
  R(0, 2) = rotation(0, 2);
  R(1, 0) = rotation(1, 0);
  R(1, 1) = rotation(1, 1);
  R(1, 2) = rotation(1, 2);
  R(2, 0) = rotation(2, 0);
  R(2, 1) = rotation(2, 1);
  R(2, 2) = rotation(2, 2);

  Mat(R).copyTo(_R);
}

void kdl2cv(const KDL::Vector &translation, cv::OutputArray _t)
{
  // get translation
  cv::Vec3d t;

  t(0) = translation[0];
  t(1) = translation[1];
  t(2) = translation[2];

  Mat(t).copyTo(_t);
}

void serialize(const cv::Point3d &in, double out[3])
{
  out[0] = in.x;
  out[1] = in.y;
  out[2] = in.z;
}

void serialize(const KDL::Rotation &R, double camera_rotation[4])
{
  R.GetQuaternion(camera_rotation[0], camera_rotation[1], camera_rotation[2], camera_rotation[3]);
}

void serialize(const KDL::Vector &translation, double camera_translation[3])
{
  camera_translation[0] = translation.data[0];
  camera_translation[1] = translation.data[1];
  camera_translation[2] = translation.data[2];
}

}
