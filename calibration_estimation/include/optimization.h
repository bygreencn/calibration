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


#include "calibration_msgs/RobotMeasurement.h"

#include "auxiliar.h"
#include "chessboard.h"
#include "conversion.h"
#include "joint_state.h"
#include "projection.h"
#include "robot_state.h"
#include "robot_state_publisher.h"
#include "calibration_msgs/RobotMeasurement.h"

namespace calib
{

class RobotState;


class Optimization
{
public:
  typedef calibration_msgs::RobotMeasurement::Ptr Msg;

  Optimization();
  ~Optimization();

  void setRobotState(RobotState *robot_state);

  void setBagData(std::vector<Msg> &msgs);

  // addData
  // deleteData

  void addMeasurement(Msg &msg);

//   generateView();

  void run();

  /// \brief Check if it is empty (valid)
//   bool empty();

//   run();
//   void showMessuaremets(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement);

protected:
  // TODO: read this information from the system.yaml
  void getCheckboardSize(const std::string &target_id, ChessBoard *cb);

  /// \brief Generate 3D chessboard corners (board_points)
  void generateCorners(Msg &msg, std::vector<cv::Point3d> *board_model_pts_3D);


private:
  RobotState *robot_state_;
  std::vector<Msg> msgs_;
};

}

#endif // OPTIMIZATION_H


