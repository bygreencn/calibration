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
  View::cameras_ = cameras;
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

  initialization();
  addResiduals();
  solver();
  updateParam();
}

void Optimization::initialization()
{
  param_camera_rot_.clear();
  param_camera_trans_.clear();

  KDL::Frame T0, current_position;
//   robot_state_->reset(); // not needed, rigid relationship between the cameras
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

  View::camera_rot_ = param_camera_rot_;
  View::camera_trans_ = param_camera_trans_;

  // create 3D points by triangulation (saved insie View: view.triang_pts_3D_)
  triangulation();
}

void Optimization::triangulation()
{
  for (size_t v = 0; v < data_->size(); v++)
  {
    // try with a subset
//     std::vector<std::string> camera_frames;
//     camera_frames.push_back("narrow_stereo_l_stereo_camera_optical_frame"); // [I|0]
//     camera_frames.push_back("narrow_stereo_r_stereo_camera_optical_frame");
//     camera_frames.push_back("wide_stereo_l_stereo_camera_optical_frame");
//     camera_frames.push_back("wide_stereo_r_stereo_camera_optical_frame");
//     camera_frames.push_back("head_mount_kinect_rgb_optical_frame");
// //     camera_frames.push_back("high_def_optical_frame");
    data_->view_[v].triangulation(cameras_, // camera_frames,
                                  param_camera_rot_,
                                  param_camera_trans_);
  }
}

void Optimization::addResiduals()
{
  // v: view index
  // i: camera index
  // j: points
  for (size_t v = 0; v < data_->size(); v++)
  {
    View &current_view = data_->view_[v];

    vector<double *> param_point_3D;

    for (size_t i = 0; i < cameras_.size(); i++)
    {
      string cam_frame = cameras_[i];

      if (i == 0)
      {
        // First camera is the reference, most be in the view
        if (!current_view.isVisible(cam_frame))
          break;

        // serialize 3D points (board points in frame 0)
        int cam_idx = current_view.getCamIdx(cameras_[i]);

        //! 3D points
        Mat board_pts_frame0 = current_view.board_transformed_pts_3D_[cam_idx];
//         Mat board_pts_frame0(current_view.triang_pts_3D_);

        serialize(board_pts_frame0, &param_point_3D);
      }
      else
      {
        if (!current_view.isVisible(cam_frame))
          continue;
      }

      // get measured_pts_2D and intrinsicMatrix
      int cam_idx = current_view.getCamIdx(cam_frame);
      vector<Point2d> &measured_pts_2D = current_view.measured_pts_2D_[cam_idx];
      Matx33d intrinsicMatrix = current_view.cam_model_[cam_idx].intrinsicMatrix();


      // feed optimazer with data
      for (int j = 0; j < measured_pts_2D.size(); j++)
      {
        ceres::CostFunction *cost_function =
          ReprojectionErrorWithQuaternions::Create(measured_pts_2D[j].x,
                                                    measured_pts_2D[j].y,
                                                    intrinsicMatrix(0,0),
                                                    intrinsicMatrix(1,1),
                                                    intrinsicMatrix(0,2),
                                                    intrinsicMatrix(1,2));

        problem_.AddResidualBlock(cost_function,
                                  NULL,                      // squared loss
                                  param_camera_rot_[i],      // camera_rot i
                                  param_camera_trans_[i],    // camera_trans i
                                  param_point_3D[j]);        // point j (constant?)
      }

      // first camera is constanst: [I|0]
      if (i == 0)
      {
        problem_.SetParameterBlockConstant(param_camera_rot_[0]);
        problem_.SetParameterBlockConstant(param_camera_trans_[0]);
      }
    }
  }
}

void Optimization::solver()
{
  // run solver
  for (size_t i = 0; i < cameras_.size(); i++)
  {
    print_array(param_camera_rot_[i], 4,   "param_camera[i]:");
    print_array(param_camera_trans_[i], 3, "               :");
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 8;
  options.max_num_iterations = 1000;
  options.function_tolerance = 1e-32;
  options.minimizer_progress_to_stdout = true;
//   options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);
  std::cout << summary.FullReport() << "\n";
  cout << "\n";

  for (size_t i = 0; i < cameras_.size(); i++)
  {
    print_array(param_camera_rot_[i],   4, "param_camera[i]:");
    print_array(param_camera_trans_[i], 3, "               :");
  }
}

void Optimization::updateParam()
{
  // get camera '0' (KDL::Frame to robot)
  KDL::Frame T0;
  robot_state_->getFK(cameras_[0], &T0);


  // i=1 (update all cameras except the first one)
  for (size_t i = 1; i < cameras_.size(); i++)
  {
    KDL::Frame frame;
    double *camera_rot = param_camera_rot_[i];
    double *camera_trans = param_camera_trans_[i];
    frame.M.Quaternion(camera_rot[0],
                       camera_rot[1],
                       camera_rot[2],
                       camera_rot[3]);

    frame.p.data[0] = camera_trans[0];
    frame.p.data[1] = camera_trans[1];
    frame.p.data[2] = camera_trans[2];



    // get father pose (father to tree root)
    const string link_root = robot_state_->getLinkRoot(cameras_[i]);
    KDL::Frame pose_father;
    robot_state_->getFK(link_root, &pose_father);

    // get cameras: [R|t]
    KDL::Frame current_position = pose_father.Inverse() * T0 * frame.Inverse();
    urdf::Pose pose;
    kdl2urdf(current_position, &pose);
    robot_state_->setUrdfPose(cameras_[i], pose);
  }

//   sleep(1);
  robot_state_->updateTree();

  View::camera_rot_ = param_camera_rot_;
  View::camera_trans_ = param_camera_trans_;


//   triangulation();
//   data_->view_[3].triangulation(cameras_, // camera_frames,
//                                 param_camera_rot_,
//                                 param_camera_trans_);
  data_->showView(3);


  //   triangulation();
//   i=4;
// //   for (size_t i=0; i<data_->size(); i++)
//   {
//     PRINT(i)
//     data_->showView(i);
//     ros::spinOnce();
//     sleep(1);
// //     ros::Duration(1).sleep(); // sleep
//   }
}

// void Optimization::calcError()
// {
//   // v: view index
//   // i: camera index
//   // j: points
//   for (size_t v = 0; v < data_->size(); v++)
//   {
//     View &current_view = data_->view_[v];
//
//     vector<double *> param_point_3D;
//
//     for (size_t i = 0; i < cameras_.size(); i++)
//     {
//       string cam_frame = cameras_[i];
//
//       int cam_idx = current_view.getCamIdx(cam_frame);
//       vector<Point2d> &measured_pts_2D = current_view.measured_pts_2D_[cam_idx];
//       Matx33d intrinsicMatrix = current_view.cam_model_[cam_idx].intrinsicMatrix();
//
//
//       // feed optimazer with data
//       for (int j = 0; j < measured_pts_2D.size(); j++)
//       {
//     }
//   }
//   }
// }

}

