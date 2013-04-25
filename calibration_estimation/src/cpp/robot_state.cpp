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

#include "robot_state.h"
#include "ros/assert.h"

using namespace std;

namespace calib
{

RobotState::RobotState()
{
}

RobotState::~RobotState()
{
}

void RobotState::initFromTree(const KDL::Tree &tree)
{
  vector<string> joint_names;

  // walk the tree recursively and add segments to segments_
  addChildren(tree.getRootSegment(), &joint_names);

  // init joint state from
  initString(joint_names);

  ROS_DEBUG("joint_names.size(): %d", joint_names.size());
  ROS_DEBUG("joint_positions_.size(): %d", (int) joint_positions_.size()); // Also included JointType==Nnoe
}

// add children to correct maps
void RobotState::addChildren(const KDL::SegmentMap::const_iterator segment,
                             vector<string> *joint_names)
{
  const vector<KDL::SegmentMap::const_iterator> &children = segment->second.children;
  for (unsigned int i = 0; i < children.size(); i++)
  {
    // find current child
    const KDL::Segment &child = children[i]->second.segment;

    // add it to segments_ (using its link name)
    segments_[child.getName()] = child;

    // associate 'joint_name' with 'link_name'
    map_[child.getJoint().getName()] = child.getName();

    // add this name to the joint_name vector
    joint_names->push_back(child.getJoint().getName());

    // continue recursively
    addChildren(children[i], joint_names);

    ROS_DEBUG("link: %s\t->\t", child.getName().c_str());
    ROS_DEBUG("joint: %s\n", child.getJoint().getName().c_str());
  }
}


bool RobotState::empty(void)
{
  return JointState::empty() || segments_.empty();
}

void RobotState::getFK(vector<KDL::Frame> &poses)
{
  // loop over all joint positions
  JointState::JointStateType::const_iterator jnt = joint_positions_.begin();
  for (; jnt != joint_positions_.end(); jnt++)
  {
    // get link name
    string &link_name = map_[jnt->first];

    // get pose
    RobotStateType::const_iterator seg = segments_.find(link_name);
    if (seg != segments_.end())
    {
      KDL::Frame pose = seg->second.pose(jnt->second);
      poses.push_back(pose);
    }
  }
}

}
