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

#include "data.h"
#include "markers.h"
#include "robot_state.h"
#include "conversion.h"
#include "projection.h"

#include "auxiliar.h"

using namespace std;
using namespace cv;

namespace calib
{

// Static member allocation
Markers    *Data::markers_ = 0;
RobotState *Data::robot_state_ = 0;

Data::Data()
{
}

Data::~Data()
{
}

void Data::setRobotState(RobotState *robot_state)
{
  robot_state_ = robot_state;
  View::setRobotState(robot_state_);
}

void Data::setMarkers(Markers *markers)
{
  markers_ = markers;
}

void Data::addMeasurement(const Msg &msg)
{
  // set and generate view from message
  View current_view;
  current_view.generateView(msg);

  // add to internal vector of views
  view_.push_back(current_view);
}

void Data::showView(std::size_t id)
{
  if (id < view_.size())
  {
    View &current_view = view_[id];
    showView(id, current_view.frame_name_);
  }
}

void Data::showView(std::size_t id, const vector<string> &camera_frames)
{
  if (id < view_.size())
  {
    markers_->reset();
    updateRobot(id);
    View &current_view = view_[id];

    cout << "View: " << id << endl;

    size_t size = camera_frames.size();
    for (size_t i = 0; i < size; i++)
    {
      // get camera index
      int cam_idx = current_view.getCamIdx(camera_frames[i]);
      if (cam_idx < 0)
        continue;

//       // points to base_footprint (new_points)
//       Mat new_points;
//       apply_transform(current_view.board_transformed_pts_3D_[cam_idx],
//                       camera_frames[i],
//                       "base_footprint",
//                       &new_points);

      // add new_points to markers_
      markers_->addMarkers(current_view.board_transformed_pts_3D_[cam_idx],
                           current_view.camera_id_[cam_idx],
                           current_view.frame_name_[cam_idx],
                           chooseColor(i));

      // show stats
      cout << "i:" << i
           << " --  camera: "           << current_view.camera_id_[cam_idx] << endl;
      cout << "\tcam_model.tfFrame(): " << current_view.frame_name_[cam_idx] << endl;
      cout << "\tReproj. error = "      << current_view.error_[cam_idx] << endl;
      cout << "\tAveg. error = "        << current_view.error_[cam_idx] / current_view.board_model_pts_3D_.size() << endl;
      cout << "\trvec = "               << current_view.rvec_[cam_idx] << endl;
      cout << "\ttvec = "               << current_view.tvec_[cam_idx] << endl << endl;
    }

    // more stats
    current_view.updateView();
    current_view.calc_error();
    current_view.output();

    // show triangulated 3D points
    if( current_view.triang_pts_3D_.size() > 0 )
    {
      // add new_points to markers_
      markers_->addMarkers(Mat(current_view.triang_pts_3D_),
                           "triangulation",
                           camera_frames[0],
                           cv::Scalar(255, 255, 255));
    }
  }
}

void Data::updateRobot(size_t id)
{
  view_[id].updateRobot();
}

void Data::apply_transform(const cv::Mat &points,
                           const string &frame,
                           cv::Mat *modif_points)
{
  KDL::Frame pose;
  robot_state_->getFK(frame, &pose);
  transform3DPoints(points, pose, modif_points);
}

void Data::apply_transform(const cv::Mat &points,
                           const string &frame1,
                           const string &frame2,
                           cv::Mat *modif_points)
{
  KDL::Frame pose1, pose2;
  robot_state_->getFK(frame1, &pose1);
  robot_state_->getFK(frame2, &pose2);
  transform3DPoints(points, pose2.Inverse() * pose1, modif_points);
}

}
