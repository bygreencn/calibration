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

#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

namespace calib
{

Optimization::Optimization()
{
  robot_state_ = 0;
  msgs_.clear();
}

Optimization::~Optimization()
{
}

void Optimization::setRobotState(RobotState *robot_state)
{
  robot_state_ = robot_state;
}

void Optimization::setBagData(vector<Msg> &msgs)
{
  msgs_ = msgs;
}

void Optimization::run()
{
  // check if the state is valid
  if( robot_state_ == 0 || msgs_.empty() )
  {
    ROS_ERROR("The optimazer data is not complete");
    return;
  }
}

void Optimization::getCheckboardSize(const string &target_id,
                                     ChessBoard *cb)
{
  if (target_id == "large_cb_7x6")
  {
    cb->setSize(7, 6, 0.108);
    return;
  }

  if (target_id == "small_cb_4x5")
  {
    cb->setSize(4, 5, 0.0245);
    return;
  }
}

void Optimization::generateCorners(Msg &msg, vector<Point3d> *board_model_pts_3D)
{
  ChessBoard cb;
  getCheckboardSize(msg->target_id, &cb);
  cb.generateCorners(board_model_pts_3D);
}

}
