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


#ifndef DATA_H
#define DATA_H

#include "view.h"

namespace calib
{

class Markers;
class RobotState;

/** Data
*
* Data manipulater: it can be inside the optimazer but it will also used
* for visualization.
*
*/
class Data
{
public:
  Data();
  ~Data();

  static void setRobotState(RobotState *robot_state);
  static void setMarkers(Markers *markers_);

  /// \brief Add one RobotMeasurement
  void addMeasurement(const Msg &msg);

  /// \brief Publish View (checkerboards, /tf, joint angles)
  void showView(std::size_t id);
  void showView(std::size_t id, const std::vector<std::string> &camera_frames);

  /// \brief Number of views
  std::size_t size() { return view_.size(); }

  /// \brief Clear views
  void clear() { view_.clear(); }

  // static members
  static Markers    *markers_;
  static RobotState *robot_state_;

  // public members
  std::vector<View> view_;

// private:
  /// \brief Update robot state with the measured joint angles
  void updateRobot(std::size_t id);

  /// \brief transform 3D using the Rotation and Translation defined in frame
  void apply_transform(const cv::Mat &points,
                       const std::string &frame,
                       cv::Mat *modif_points);

  /// \brief transform 3D from frame1 to frame2
  void apply_transform(const cv::Mat &points,
                       const std::string &frame1,
                       const std::string &frame2,
                       cv::Mat *modif_points);
};

}

#endif // DATA_H
