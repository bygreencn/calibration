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
#include "chessboard.h"

#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

namespace calib
{

Optimization::Optimization()
{
  robot_state_ = 0;
  msgs_ = 0;
  markers_ = 0;
}

Optimization::~Optimization()
{
}

void Optimization::setRobotState(RobotState *robot_state)
{
  robot_state_ = robot_state;
}

void Optimization::setBagData(vector<Msg> *msgs)
{
  msgs_ = msgs;
}

void Optimization::setMarkers(Markers *markers)
{
  markers_ = markers;
}

bool Optimization::valid()
{
  return robot_state_ != 0 && msgs_ != 0 && markers_ == 0;
}

void Optimization::run()
{
  // check if the state is valid
  if( !valid() )
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

void Optimization::generateCorners(const Msg &msg,
                                   vector<Point3d> *board_model_pts_3D)
{
  ChessBoard cb;
  getCheckboardSize(msg->target_id, &cb);
  cb.generateCorners(board_model_pts_3D);
}

void Optimization::getCameraModels(const Msg &msg,
                                   vector<image_geometry::PinholeCameraModel> *cams)
{
  // clear Camera model vector (cams)
  cams->clear();

  // get Camera models
  size_t size = msg->M_cam.size();
  for (size_t i = 0; i < size; i++)
  {
    // get camera info
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(msg->M_cam.at(i).cam_info);

    // add to vector
    cams->push_back(cam_model);
  }
}

void Optimization::getMeasurements(const Msg &msg,
                                   std::vector<View::Points2D> *measurements)
{
  // clear measurements vector
  measurements->clear();

  size_t size = msg->M_cam.size();
  for (size_t i = 0; i < size; i++)
  {
    // get measurement
    const vector<geometry_msgs::Point> &pts_ros = msg->M_cam.at(i).image_points;

    // convert ROS points to OpenCV
    vector<Point3d> pts;
    ros2cv(pts_ros, &pts);

    // remove last rows (this message has xyz values, with z=0 for camera)
    vector<Point2d> measured_pts_2D;
    measured_pts_2D.resize(pts.size());
    for(int j=0; j < pts.size(); j++)
    {
      measured_pts_2D[j].x = pts[j].x;
      measured_pts_2D[j].y = pts[j].y;
    }

    // add to vector
    measurements->push_back(measured_pts_2D);
  }
}

void Optimization::findCbPoses(const Msg &msg,
                               const vector<Point3d> &board_model_pts_3D,
                               const vector<View::Points2D> &measured_pts_2D,
                               const vector<Mat> &intrinsicMatrix,
                               vector<Mat> *_rvec,
                               vector<Mat> *_tvec,
                               vector<View::Points2D> *_expected_pts_2D,
                               vector<double> *_error)
{
  // clean vectors
  _rvec->clear();
  _tvec->clear();
  _expected_pts_2D->clear();
  _error->clear();

  size_t size = measured_pts_2D.size();
  for (size_t i = 0; i < size; i++)
  {
    Mat rvec, tvec;
    View::Points2D expected_pts_2D;
    Mat D; // empty for rectified cameras
    // Mat D = cam_model.distortionCoeffs();  // for non-rectified cameras
    double error = findChessboardPose(board_model_pts_3D, measured_pts_2D[i],
                                      intrinsicMatrix[i], D,
                                      rvec, tvec, expected_pts_2D);

    // add to vector
    _rvec->push_back(rvec);
    _tvec->push_back(tvec);
    _expected_pts_2D->push_back(expected_pts_2D);
    _error->push_back(error);
  }
}

void Optimization::getFrameNames(const Msg &msg,
                                 std::vector<std::string> *frame,
                                 std::map<std::string, std::size_t> *frame_id)
{
  for (size_t i = 0; i < msg->M_cam.size(); i++)
  {
    string current_frame;

    // TODO: some frame names are hard-coded here.
    // This is a possible error in the bag file
    if( msg->M_cam.at(i).camera_id == "narrow_left_rect" )
      current_frame = "narrow_stereo_l_stereo_camera_optical_frame";
    else if( msg->M_cam.at(i).camera_id == "narrow_right_rect" )
      current_frame = "narrow_stereo_r_stereo_camera_optical_frame";
    else if( msg->M_cam.at(i).camera_id == "wide_left_rect" )
      current_frame = "wide_stereo_l_stereo_camera_optical_frame";
    else if( msg->M_cam.at(i).camera_id == "wide_right_rect" )
      current_frame = "wide_stereo_r_stereo_camera_optical_frame";
    else if( msg->M_cam.at(i).camera_id == "kinect_head" )
      current_frame = "head_mount_kinect_rgb_optical_frame";
    else if( msg->M_cam.at(i).camera_id == "prosilica_rect" )
      current_frame = "high_def_optical_frame";
    else
    {
      // get camera info
      image_geometry::PinholeCameraModel cam_model;
      cam_model.fromCameraInfo(msg->M_cam.at(i).cam_info);
      current_frame = cam_model.tfFrame();
    }

    // add to vector
    frame->push_back(current_frame);

    // generate map
    (*frame_id)[current_frame] = i;
  }
}

}



