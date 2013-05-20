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
#include "markers.h"
#include "projection.h"
#include "robot_state.h"
#include "robot_state_publisher.h"
#include "calibration_msgs/RobotMeasurement.h"

namespace calib
{

class RobotState;

struct View
{
  typedef std::vector<cv::Point3d> Points3D;
  typedef std::vector<cv::Point2d> Points2D;

  unsigned id;

  Points3D              board_model_pts_3D;  // generateCorners
  std::vector<Points2D> measured_pts_2D;     // getMeasurement
  std::vector<Points2D> expected_pts_2D;     // findCbPoses
  std::vector<double>   error;               // findCbPoses

  std::vector<image_geometry::PinholeCameraModel> cam_model;
//   unsigned cam_number;  cam_model.size()

  std::vector<std::string>           frame_name;
  std::map<std::string, std::size_t> frame_id;   // frame_name -> frame_id

  std::vector<KDL::Frame> pose_rel, pose_father;
  KDL::Frame T0; // T0 == pose_father[0]*pose_rel[0]
};

class Optimization
{
public:
  typedef calibration_msgs::RobotMeasurement::Ptr Msg;

  Optimization();
  ~Optimization();

  // Set optimizer funtions
  void setRobotState(RobotState *robot_state);
  void setBagData(std::vector<Msg> *msgs);
  void setMarkers(Markers *markers);


  // addData
  // deleteData

  void addMeasurement(Msg &msg);

//   generateView();

  bool valid();

  void run();

  /// \brief Check if it is empty (valid)
//   bool empty();

//   run();
//   void showMessuaremets(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement);

protected:
  // TODO: read this information from the system.yaml
  void getCheckboardSize(const std::string &target_id,
                         ChessBoard *cb);

  /// \brief Generate 3D chessboard corners (board_points)
  void generateCorners(const Msg &msg,
                       std::vector<cv::Point3d> *board_model_pts_3D);

  /// \brief Get Camera Model from Message
  void getCameraModels(const Msg &msg,
                      std::vector<image_geometry::PinholeCameraModel> *cams);


  /// \brief Read image points from message
  void getMeasurements(const Msg &msg,
                       std::vector<View::Points2D> *measurements);

  /// \brief Find chessboard poses using solvePnP
  void findCbPoses(const Msg &msg,
                   const std::vector<cv::Point3d> &board_model_pts_3D,
                   const std::vector<View::Points2D> &measured_pts_2D,
                   const std::vector<cv::Mat> &intrinsicMatrix,
                   std::vector<cv::Mat> *rvec,
                   std::vector<cv::Mat> *tvec,
                   std::vector<View::Points2D> *expected_pts_2D,
                   std::vector<double> *error);


  void getFrameNames(const Msg &msg,
                     std::vector<std::string> *frame,
                     std::map<std::string, std::size_t> *frame_id);



private:
  RobotState        *robot_state_;
  std::vector<Msg>  *msgs_;
  Markers           *markers_;
};

}

#endif // OPTIMIZATION_H


