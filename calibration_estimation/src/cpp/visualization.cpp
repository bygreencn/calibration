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
#include <visualization_msgs/Marker.h>
#include <opencv2/calib3d/calib3d.hpp>

#include "auxiliar.h"
#include "chessboard.h"
#include "conversion.h"
#include "joint_state.h"
#include "projection.h"
#include "calibration_msgs/RobotMeasurement.h"

using namespace std;
using namespace calib;

// global variables
JointState joint_state;
ros::Publisher joint_pub;
ros::Publisher vis_pub;
robot_state_publisher::RobotStatePublisher *robot_st_publisher;
map<string, double> joint_positions;

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

void print_mat(cv::Mat tmp)
{
  cout << "size: " << tmp.rows << "x" << tmp.cols << endl;
  cout << tmp/*.t()*/ << endl;
}

// find chessboard pose using solvePnP
// TODO: move to another place
double findChessboardPose(cv::InputArray objectPoints,
                          cv::InputArray imagePoints,
                          cv::InputArray cameraMatrix,
                          cv::InputArray distCoeffs,
                          cv::OutputArray rvec,
                          cv::OutputArray tvec,
                          cv::OutputArray proj_points2D =cv::noArray())
{
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
               rvec, tvec, false, CV_ITERATIVE);

  // reprojection error
  double err = computeReprojectionErrors(objectPoints, imagePoints,
                                         cameraMatrix, distCoeffs,
                                         rvec, tvec, proj_points2D);

  return err;
}

void setMarkersParam(const string &frame,
                     visualization_msgs::Marker *marker,
                     double scale =0.02,
                     double r =0.0,
                     double g =1.0,
                     double b =1.0)
{
  marker->header.frame_id = frame;
  marker->header.stamp = ros::Time();
  marker->ns = "calib";
  marker->id = 0;
  marker->type = visualization_msgs::Marker::CUBE_LIST;
//   marker->type = visualization_msgs::Marker::SPHERE_LIST;
  marker->action = visualization_msgs::Marker::ADD;

  marker->scale.x = scale;
  marker->scale.y = scale;
  marker->scale.z = scale;

  marker->color.r = r;
  marker->color.g = g;
  marker->color.b = b;
  marker->color.a = 1.0;
}

void points2markers(const cv::Mat board_measured_pts_3D,
                    visualization_msgs::Marker *marker)
{
  cv2ros(board_measured_pts_3D, &(marker->points));
}


// using namespace cv;

void showMessuaremets(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement)
{
  // generate 3D chessboard corners (board_points)
  ChessBoard cb;
  getCheckboardSize(robot_measurement->target_id, &cb);
  vector<cv::Point3d> board_model_pts_3D;
  cb.generateCorners(&board_model_pts_3D);

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
    vector<cv::Point2d> measured_pts_2D;
    measured_pts_2D.resize(pts.size());
    for(int i=0; i < pts.size(); i++)
    {
      measured_pts_2D[i].x = pts[i].x;
      measured_pts_2D[i].y = pts[i].y;
    }

    // find chessboard pose using solvePnP
    cv::Mat rvec, tvec;
    vector<cv::Point2d> expected_pts_2D;
    double err = findChessboardPose(board_model_pts_3D, measured_pts_2D,
                                    cam_model.intrinsicMatrix(),
                                    cam_model.distortionCoeffs(),
                                    rvec, tvec, expected_pts_2D);

    // board_measured_pts
    cv::Mat board_measured_pts_3D;
    project3dPoints(cv::Mat(board_model_pts_3D), rvec, tvec, &board_measured_pts_3D);
//     cout << "\tboard_measured_pts = " << board_measured_pts_3D << endl;

//     double err2 = cv::norm(found_board_corners, proj_points2D, CV_L2);

// /// calculate expected checkboard, in order to compare 3D points
//     double err2 = findChessboardPose(board_model_pts_3D, expected_pts_2D,
//                                      cam_model.intrinsicMatrix(),
//                                      cam_model.distortionCoeffs(),
//                                      rvec, tvec);
// 
//     // board_expected_pts
//     cv::Mat board_expected_pts;
//     project3dPoints(cv::Mat(board_model_pts_3D), rvec, tvec, &board_expected_pts);
//     cout << "\tboard_measured_pts = " << endl;
//     print_mat(board_expected_pts);


    // checkboard visualization
    visualization_msgs::Marker marker;
    setMarkersParam(cam_model.tfFrame(), &marker);
    points2markers(board_measured_pts_3D, &marker);
    vis_pub.publish(marker);


//     // TODO: find another way to calculate the repojection error from 3D points
//     projectPoints(cam_model, modif_points, &x2d_proj);
//     double err2 = norm(found_board_corners, x2d_proj, CV_L2);

    cout << "\tReproj. err = "  << err << endl;
//     cout << "\tReproj. err2 = " << err2 << endl;
    cout << "\trvec = " << rvec << endl;
    cout << "\ttvec ="  << tvec << endl << endl;
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
  showMessuaremets(robot_measurement);

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
  ros::NodeHandle n; //("calib");

  // subscriber
  ros::Subscriber subs_robot_measurement = n.subscribe("robot_measurement", 1,
                                                       robotMeasurementCallback);
  // publisher
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  // visualization marker publisher
  vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  ros::spin();

  delete robot_st_publisher;
  return 0;
}

