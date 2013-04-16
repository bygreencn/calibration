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

/**
 * This is a app for in visualizing errors
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>

#include "auxiliar.h"
#include "chessboard.h"
#include "joint_state.h"
#include "calibration_msgs/RobotMeasurement.h"

using namespace std;
using namespace calib;

// global variables
JointState joint_state;
ros::Publisher joint_pub;
robot_state_publisher::RobotStatePublisher *robot_st_publisher;
map<string, double> joint_positions;

// Print vector<T>: cout << vector<T>
template<typename T>
std::ostream &operator <<(std::ostream &os, const std::vector<T> &v)
{
  std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
  return os;
}

bool initJoinStateFromParam(JointState *joint_state)
{
  vector<string> names;
  if (!getJoinNamesFromParam("robot_description", &names))
    return false;

  joint_state->initString(names);

  return true;
}

bool readRobotDescription(const string &param,
                          KDL::Tree *kdl_tree)
{
  ros::NodeHandle node;
  std::string robot_desc_string;
  node.param(param, robot_desc_string, string());

  if (!kdl_parser::treeFromString(robot_desc_string, *kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

//   // Get segments
//   KDL::SegmentMap robot_segments = robot_kdl_tree.getSegments();
//
//   // Print names
//   KDL::SegmentMap::const_iterator it = robot_segments.begin();
//
//   for (unsigned i = 0; it != robot_segments.end(); it++, i++)
//   {
//     cout << it->first << endl;
//   }

  return true;
}

void ros2cv(const geometry_msgs::Point &pt_ros, cv::Point3d *pt_cv)
{
  pt_cv->x = pt_ros.x;
  pt_cv->y = pt_ros.y;
  pt_cv->z = pt_ros.z;
}

void ros2cv(const vector<geometry_msgs::Point> &pt_ros, vector<cv::Point3d> *pt_cv)
{
  pt_cv->clear();
  pt_cv->reserve(pt_ros.size());

  vector<geometry_msgs::Point>::const_iterator it = pt_ros.begin();
  for(; it < pt_ros.end(); ++it)
  {
    cv::Point3d current_pt;
    ros2cv(*it,&current_pt);
    pt_cv->push_back(current_pt);
  }
}

void ros2cv(const vector<geometry_msgs::Point> &pt_ros, cv::Mat_<double> *pt_cv)
{
  vector<cv::Point3d> points;
  ros2cv(pt_ros, &points);
  *pt_cv = cv::Mat(points);
}


void projectPoints(const image_geometry::PinholeCameraModel &cam_model,
                   const cv::Point3d &xyz,
                   cv::Point2d *points2D)
{
  *points2D = cam_model.project3dToPixel(xyz);
//   *points2D = cam_model.rectifyPoint(*points2D);
}

void projectPoints(const image_geometry::PinholeCameraModel &cam_model,
                   const vector<cv::Point3d> &xyz,
                   vector<cv::Point2d> *points2D)
{
  size_t n = xyz.size();
  points2D->clear();
  points2D->reserve(n);

  for(size_t i=0; i<n; i++)
  {
    cv::Point2d current_pt;
    projectPoints(cam_model, xyz[i], &current_pt);
    points2D->push_back(current_pt);
  }
}

// ToDo: read this information from the system.yaml
void getCheckboardSize(const string &target_id, ChessBoard *cb)
{
  if (target_id == "large_cb_7x6")
  {
    cb->setSize( 7, 6, 0.108 );
    return;
  }

  if (target_id == "small_cb_4x5")
  {
    cb->setSize( 4, 5, 0.0245 );
    return;
  }
}

void show_messuaremets(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement)
{
  // generate 3D chessboard corners (board_points)
  ChessBoard cb;
  getCheckboardSize(robot_measurement->target_id, &cb);
  vector<cv::Point3d> board_points;
  cb.generateCorners(&board_points);


  unsigned size = robot_measurement->M_chain.size();
  unsigned i = 0;
//   for (unsigned i = 0; i < size; i++)
  {
    // get camera info
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(robot_measurement->M_cam.at(i).cam_info);

    // get measurement
    const vector<geometry_msgs::Point> &pts_ros = robot_measurement->M_cam.at(i).image_points;

    // convert ROS points to OpenCV
    std::vector<cv::Point3d> pts;
    ros2cv(pts_ros, &pts);

    // remove last rows
    vector<cv::Point2d> found_board_corners;
    found_board_corners.resize(pts.size());
    for(int i=0; i < pts.size(); i++)
    {
      found_board_corners[i].x = pts[i].x;
      found_board_corners[i].y = pts[i].y;
    }

    // find chessboard pose using solvePnP
    cv::Mat rvec, tvec;
    cv::solvePnP(board_points, found_board_corners,
                 cam_model.intrinsicMatrix(),
                 cam_model.distortionCoeffs(),
                 rvec, tvec, false, CV_ITERATIVE);

    cout<<" rvec "<<endl<<" "<<rvec<<endl;
    cout<<" tvec "<<endl<<" "<<tvec<<endl<<endl;
  }

}

void robotMeasurementCallback(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement)
{
  // reset joints to zeros
  joint_state.reset();

  // update joints
  unsigned size = robot_measurement->M_chain.size();
  for (unsigned i = 0; i < size; i++)
  {
    joint_state.update(robot_measurement->M_chain.at(i).chain_state.name,
                       robot_measurement->M_chain.at(i).chain_state.position);
  }

  // publish moving joints
  robot_st_publisher->publishTransforms(joint_state.getJointPositions(),
                                        ros::Time::now());

  // show messuaremets
  show_messuaremets(robot_measurement);

  // calculate expected values

  // publish joints
//   publishJoints(joint_pub, joint_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualization");

  if (!initJoinStateFromParam(&joint_state))
  {
    return 0;
  }

  // read robot description and create robot_state_publisher
  KDL::Tree kdl_tree;
  readRobotDescription("robot_description", &kdl_tree);
  if (!robot_st_publisher)
    robot_st_publisher = new robot_state_publisher::RobotStatePublisher(kdl_tree);

  // create node
  ros::NodeHandle n;

  // subscriber
  ros::Subscriber subs_robot_measurement = n.subscribe("robot_measurement", 1,
                                                       robotMeasurementCallback);
  // publisher
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::spin();

  delete robot_st_publisher;
  return 0;
}

