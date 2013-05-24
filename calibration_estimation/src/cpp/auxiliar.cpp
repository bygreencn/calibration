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

#include "auxiliar.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace ros;
using namespace cv;

namespace calib
{

void print_array(double *array, unsigned size, const std::string &msg)
{
  cv::Mat_<double> out(size, 1, array);
  std::cout << msg << out << std::endl;
}



double norm(const vector<Point2d> &p1,
            const vector<Point2d> &p2,
            vector<double> *ind_error)
{
  size_t size = p1.size();
  assert(size && p2.size());

  if (ind_error != 0)
  {
    ind_error->clear();
    ind_error->resize(size);
  }

  double error = 0;
  for (int i = 0; i < p1.size(); i++)
  {
    Point2d diff = p1[i] - p2[i];
    double current_error = sqrt(diff.dot(diff));
    error += current_error;

//     double x_diff = p1[i].x - p2[i].x;
//     double y_diff = p1[i].y - p2[i].y;
//     error += sqrt(x_diff*x_diff + y_diff*y_diff);

    if (ind_error != 0)
      (*ind_error)[i] = current_error;
  }

  return error;
}


}
