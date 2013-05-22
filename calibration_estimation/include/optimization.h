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

#include "data.h"
#include "cost_functions.h"

namespace calib
{

class RobotState;
class Markers;

class Optimization
{
public:
  Optimization();
  ~Optimization();

  /// \brief Set optimizer funtions
  void setRobotState(RobotState *robot_state);
  void setMarkers(Markers *markers);
  void setData(Data *data);

  /// \brief Set vector of 'cameras_id' to be calibrated
  void setCamerasCalib(const std::vector<std::string> &cameras);

  /// \brief Check is the state is valid
  bool valid();

  /// \brief Run optimization process
  void run();

private:
  void initialization();

private:
  RobotState *robot_state_;
  Markers    *markers_;
  Data       *data_;

  std::vector<std::string> cameras_;  // camera to be calibrated

  ceres::Problem problem_;

  std::vector<std::vector<double *> > param_point_3D_;
  std::vector<double *>               param_camera_rot_;
  std::vector<double *>               param_camera_trans_;
};

}

#endif // OPTIMIZATION_H


