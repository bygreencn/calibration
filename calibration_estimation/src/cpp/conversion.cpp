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

using namespace std;
using namespace cv;

namespace calib
{

void ros2cv(const geometry_msgs::Point &pt_ros, Point3d *pt_cv)
{
  pt_cv->x = pt_ros.x;
  pt_cv->y = pt_ros.y;
  pt_cv->z = pt_ros.z;
}

void ros2cv(const vector<geometry_msgs::Point> &pt_ros, vector<Point3d> *pt_cv)
{
  pt_cv->clear();
  pt_cv->reserve(pt_ros.size());

  vector<geometry_msgs::Point>::const_iterator it = pt_ros.begin();
  for(; it < pt_ros.end(); ++it)
  {
    Point3d current_pt;
    ros2cv(*it,&current_pt);
    pt_cv->push_back(current_pt);
  }
}

void ros2cv(const vector<geometry_msgs::Point> &pt_ros, Mat_<double> *pt_cv)
{
  vector<Point3d> points;
  ros2cv(pt_ros, &points);
  *pt_cv = Mat(points);
  transpose(*pt_cv, *pt_cv);
}

}
