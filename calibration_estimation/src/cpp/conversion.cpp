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

}
