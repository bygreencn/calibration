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

#include "view.h"
#include "chessboard.h"
#include "conversion.h"

using namespace std;
using namespace cv;

namespace calib
{

View::View()
{
}

View::~View()
{
}

void View::generateCorners()
{
  ChessBoard cb;
  getCheckboardSize(msg_->target_id, &cb);
  cb.generateCorners(&board_model_pts_3D_);
}

void View::getCameraModels()
{
  // clear Camera model vector (cams)
  cam_model_.clear();

  // get Camera models
  size_t size = msg_->M_cam.size();
  for (size_t i = 0; i < size; i++)
  {
    // get camera info
    image_geometry::PinholeCameraModel current_cam_model;
    current_cam_model.fromCameraInfo(msg_->M_cam.at(i).cam_info);

    // add to vector
    cam_model_.push_back(current_cam_model);
  }
}

void View::getMeasurements()
{
  // clear measurements vector
  measured_pts_2D_.clear();

  size_t size = msg_->M_cam.size();
  for (size_t i = 0; i < size; i++)
  {
    // get measurement
    const vector<geometry_msgs::Point> &pts_ros = msg_->M_cam.at(i).image_points;

    // convert ROS points to OpenCV
    vector<Point3d> pts;
    ros2cv(pts_ros, &pts);

    // remove last rows (this message has xyz values, with z=0 for camera)
    vector<Point2d> current_measured_pts_2D;
    current_measured_pts_2D.resize(pts.size());
    for(int j=0; j < pts.size(); j++)
    {
      current_measured_pts_2D[j].x = pts[j].x;
      current_measured_pts_2D[j].y = pts[j].y;
    }

    // add to vector
    measured_pts_2D_.push_back(current_measured_pts_2D);
  }
}

void View::findCbPoses(const vector<Point3d> &board_model_pts_3D,
                       const vector<View::Points2D> &measured_pts_2D,
                       const vector<Mat> &intrinsicMatrix)
{
  // clean vectors
  rvec_.clear();
  tvec_.clear();
  expected_pts_2D_.clear();
  error_.clear();

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
    rvec_.push_back(rvec);
    tvec_.push_back(tvec);
    expected_pts_2D_.push_back(expected_pts_2D);
    error_.push_back(error);
  }
}

void View::getFrameNames()
{
  frame_name_.clear();
  frame_id_.clear();

  for (size_t i = 0; i < msg_->M_cam.size(); i++)
  {
    string current_frame;

    // TODO: some frame names are hard-coded here.
    // This is a possible error in the bag file
    if( msg_->M_cam.at(i).camera_id == "narrow_left_rect" )
      current_frame = "narrow_stereo_l_stereo_camera_optical_frame";
    else if( msg_->M_cam.at(i).camera_id == "narrow_right_rect" )
      current_frame = "narrow_stereo_r_stereo_camera_optical_frame";
    else if( msg_->M_cam.at(i).camera_id == "wide_left_rect" )
      current_frame = "wide_stereo_l_stereo_camera_optical_frame";
    else if( msg_->M_cam.at(i).camera_id == "wide_right_rect" )
      current_frame = "wide_stereo_r_stereo_camera_optical_frame";
    else if( msg_->M_cam.at(i).camera_id == "kinect_head" )
      current_frame = "head_mount_kinect_rgb_optical_frame";
    else if( msg_->M_cam.at(i).camera_id == "prosilica_rect" )
      current_frame = "high_def_optical_frame";
    else
    {
      // get camera info
      image_geometry::PinholeCameraModel cam_model;
      cam_model.fromCameraInfo(msg_->M_cam.at(i).cam_info);
      current_frame = cam_model.tfFrame();
    }

    // add to vector
    frame_name_.push_back(current_frame);

    // generate map
    frame_id_[current_frame] = i;
  }
}

void View::getPoses(RobotState *robot_state_)
{
  for (size_t i = 0; i < msg_->M_cam.size(); i++)
  {
    // get relative pose (camera to its father)
    KDL::Frame pose;
    robot_state_->getRelativePose(frame_name_[i], &pose);
    pose_rel_.push_back(pose);
    const string link_root = robot_state_->getLinkRoot(frame_name_[i]);

    // get father pose (father to tree root)
    KDL::Frame pose_f;
    robot_state_->getFK(link_root, &pose_f);
    pose_father_.push_back(pose_f);
  }
}

}
