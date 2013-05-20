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


#ifndef VIEW_H
#define VIEW_H

#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
// #include <image_geometry/stereo_camera_model.h>

#include "calibration_msgs/RobotMeasurement.h"
#include "robot_state.h"

namespace calib
{

typedef calibration_msgs::RobotMeasurement::Ptr Msg;

/** View
*
* Container of all the information obtained from a checkerboard sighting
* It is generated from a Msg message (a robot measurement)
*
*/
class View
{
public:
  typedef std::vector<cv::Point3d> Points3D;
  typedef std::vector<cv::Point2d> Points2D;

  View();
  ~View();

  void setRobotState(RobotState *robot_state);

  /// \brief Generate View from RobotMeasurement Message
  bool generateView(Msg &msg);


// public Members
  Msg msg_;  // most of the rest public member are generated from msg_

  Points3D              board_model_pts_3D_;  // generateCorners
  std::vector<Points2D> measured_pts_2D_;     // getMeasurement

  std::vector<Points2D> expected_pts_2D_;     // findCbPoses
  std::vector<double>   error_;               // findCbPoses
  std::vector<cv::Mat>  rvec_;                // findCbPoses
  std::vector<cv::Mat>  tvec_;                // findCbPoses

  std::vector<image_geometry::PinholeCameraModel> cam_model_;
//   unsigned cam_number_;  cam_model_.size()

  std::vector<std::string>           frame_name_;
  std::map<std::string, std::size_t> frame_id_;   // frame_name -> frame_id

  std::vector<KDL::Frame> pose_rel_, pose_father_;
//   KDL::Frame T0; // T0 == pose_father_[0]*pose_rel_[0]


private:
  /// \brief Generate 3D chessboard corners (board_points)
  void generateCorners();

  /// \brief Get Camera Model from Message
  void getCameraModels();

  /// \brief Read image points from Message
  void getMeasurements();

  /// \brief Find chessboard poses using solvePnP
  void findCbPoses();

  /// \brief Get camera name frame from Message
  void getFrameNames();

  /// \brief Get Poses from Message and using robot_state_ for calculating the FK
  void getPoses();


  RobotState *robot_state_; // it is needed in order to calculate cameras poses
};

}

#endif // VIEW_H

