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
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>

#include "auxiliar.h"
#include "joint_state.h"
#include "calibration_msgs/RobotMeasurement.h"

using namespace std;
using namespace calib;

// global variables
JointState joint_state;
ros::Publisher joint_pub;

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

bool read_robot_description()
{
  KDL::Tree robot_kdl_tree;
  ros::NodeHandle node;
  std::string robot_desc_string;
  node.param("robot_description", robot_desc_string, string());

  if (!kdl_parser::treeFromString(robot_desc_string, robot_kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  // Get segments
  KDL::SegmentMap robot_segments = robot_kdl_tree.getSegments();

  // Print names
  KDL::SegmentMap::const_iterator it = robot_segments.begin();

  for (unsigned i = 0; it != robot_segments.end(); it++, i++)
  {
    cout << it->first << endl;
  }

  return true;
}

void publish_joints(const JointState &joint_state)
{
  // message declaration
  sensor_msgs::JointState joint_state_msg;
  
  // set stamp
  joint_state_msg.header.stamp = ros::Time::now();

  // set joint names
  joint_state_msg.name = joint_state.getJointNames();

  // set positions
  joint_state_msg.position = joint_state.getJointPositions();

  // Publish Joint state
//   ROS_INFO("[visualization_node] publish_joins");  // TODO: ROS_INFO doesn't work
  cout << "[visualization_node] publish_joins\n";

  cout << joint_state_msg.position << endl;
  joint_pub.publish(joint_state_msg);
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

  // publish joints
  publish_joints(joint_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  if (!initJoinStateFromParam(&joint_state))
  {
    return 0;
  }

  ros::NodeHandle n;
  ros::Subscriber subs_robot_measurement;
//   ros::Subscriber sub = n.subscribe("joint_states", 1, createLookUpTableCallback);
//   read_robot_description();
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  subs_robot_measurement = n.subscribe("robot_measurement", 1, robotMeasurementCallback);

  ros::spin();

  return 0;
}

