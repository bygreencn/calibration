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

#include "optimization.h"
#include "robot_state.h"
#include "markers.h"
#include "conversion.h"

#include "auxiliar.h"

#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

namespace calib
{

Optimization::Optimization()
{
  robot_state_ = 0;
  markers_ = 0;
  data_ = 0;
  cameras_.clear();
}

Optimization::~Optimization()
{
}

void Optimization::setRobotState(RobotState *robot_state)
{
  robot_state_ = robot_state;
}

void Optimization::setMarkers(Markers *markers)
{
  markers_ = markers;
}

void Optimization::setData(Data *data)
{
  data_        = data;
//   robot_state_ = Data::robot_state_;
//   markers_     = Data::markers_;
}

void Optimization::setCamerasCalib(const std::vector<std::string> &cameras)
{
  cameras_ = cameras;
}

bool Optimization::valid()
{
  return robot_state_ != 0 && markers_ != 0 && data_ != 0;
}

void Optimization::run()
{
  // check if the state is valid
  if( !valid() )
  {
    ROS_ERROR("The optimazer data is not complete");
    return;
  }

  // v: view index
  // c: camera index
  size_t v=3;
//   for (size_t v = 0; v < data_->size(); v++)
  {
//     View &current_view = data_->view_[v];
//
//     KDL::Frame T0; // (camera '0' KDL::Frame to robot)
//     for (size_t c = 0; c < cameras_.size(); c++)
//     {
//       string current_camera_id = cameras_[c];
//
// //       PRINT(current_camera_id);
// //       cout << "current_view.camera_id_: " << current_view.camera_id_ << endl;
//
//       if(c==0)
//       {
//
//         // First camera is the refence, most be in the view
//         if( !current_view.isVisible(current_camera_id) )
//           break;
//
//         int cam_idx = current_view.getCamIdx(current_camera_id);
//         T0 = current_view.pose_father_[cam_idx] * current_view.pose_rel_[cam_idx];
//       }
//       else
//       {
//         if( !current_view.isVisible(current_camera_id) )
//           continue;
//       }
//
//       int cam_idx = current_view.getCamIdx(current_camera_id);
//       KDL::Frame Ti = current_view.pose_father_[cam_idx] * current_view.pose_rel_[cam_idx];
//       KDL::Frame current_position = (Ti).Inverse() * T0;
//     }
  }
}

void Optimization::initialization()
{
  KDL::Frame T0, current_position;
  // robot_state_->reset(); // not needed, rigid relationship between the cameras
  for (int c = 0; c < cameras_.size(); c++)
  {
    // get camera pose (camera to robot root)
    KDL::Frame Ti;
    robot_state_->getFK(cameras_[c], &Ti);

    // Initial camera
    if (c == 0)
      T0 = Ti;

    // getting [R\t]
    current_position = Ti.Inverse() * T0;

    // generate cameras in doubles
    double *camera_rot = new double[4];
    serialize(current_position.M, camera_rot);
    param_camera_rot_.push_back(camera_rot);

    double *camera_trans = new double[3];
    serialize(current_position.p, camera_trans);
    param_camera_trans_.push_back(camera_trans);
  }
}

}
