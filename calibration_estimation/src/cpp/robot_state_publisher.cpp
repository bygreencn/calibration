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

#include "robot_state_publisher.h"
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include "ros/assert.h"


using namespace std;

namespace calib
{

RobotStatePublisher::RobotStatePublisher()
{
  // set publish frequency
  double publish_freq;
  node_.param("publish_frequency", publish_freq, 50.0);

  // trigger to publish fixed joints
  publish_interval_ = ros::Duration(1.0/max(publish_freq,1.0));
  timer_ = node_.createTimer(publish_interval_, &RobotStatePublisher::publishTransforms, this);
}

RobotStatePublisher::~RobotStatePublisher()
{
}

bool RobotStatePublisher::update(const string &joint_name,
                                 const double &position)
{
  // using RobotState implementation
  bool result = RobotState::update(joint_name, position);

  // publishing transforms right away
  const ros::TimerEvent dummy_event;
  publishTransforms(dummy_event);

  return result;
}

bool RobotStatePublisher::update(const vector<string> &joint_name,
                                 const vector<double> &position)
{
  // using RobotState implementation
  bool result = RobotState::update(joint_name, position);

  // publishing transforms right away
  const ros::TimerEvent dummy_event;
  publishTransforms(dummy_event);

  return result;
}

void RobotStatePublisher::publishTransforms(const ros::TimerEvent &e)
{
//   ROS_DEBUG("Publishing transforms for moving joints");

  vector<tf::StampedTransform> tf_transforms;
  tf::StampedTransform tf_transform;

  ros::Time now   = ros::Time::now();
  ros::Time delay = now + ros::Duration(0.5);

  // loop over all joint positions
  JointState::JointStateType::const_iterator jnt = joint_positions_.begin();
  for (; jnt != joint_positions_.end(); jnt++)
  {
    // get pose
    KDL::Frame pose;
    const string &link_name = getLinkName(jnt->first);

    if (getJointType(jnt->first) == KDL::Joint::None)
    {
      // fixed Transforms
      tf_transform.stamp_ = delay;  // future publish by 0.5 seconds
      getPose(link_name, 0, &pose);
    }
    else
    {
      // moving transforms
      tf_transform.stamp_ = now;
      getPose(link_name, jnt->second, &pose);
    }

    // convert KDL::Frame to tf::Transform
    tf::transformKDLToTF(pose, tf_transform);

    // complete tf::Transform information
    tf_transform.frame_id_ = getRoot(link_name);
    tf_transform.child_frame_id_ = link_name;

    tf_transforms.push_back(tf_transform);
  }

  tf_broadcaster_.sendTransform(tf_transforms);
}

}
