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
#include <kdl_parser/kdl_parser.hpp>
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

void RobotState::initFromURDF(const urdf::Model &model)
{
  // copy urdf model
  urdf_model_ = model;

  // create internal structures
  initSegments();

  // get joint names
  vector<string> joint_names;
  getJointNames(&joint_names);

  // init joint_state (vector of angles joints)
  JointState::initString(joint_names);
}

void RobotState::getRoot(const string &link_name, string *root) const
{
  root->clear();

  const urdf::Link *link = urdf_model_.getLink(link_name).get();
  if (!link)
  {
    ROS_ERROR("Link name could not been found");
    return;
  }

  const urdf::Link *parent = link->getParent().get();
  if (!parent)
  {
    ROS_ERROR("Parent link name could not been found");
    return;
  }

  *root = parent->name;
}

void RobotState::getJointNames(vector<string> *joint_name) const
{
  joint_name->clear();

  // get joint names
  map<string, boost::shared_ptr<urdf::Joint> >::const_iterator it = urdf_model_.joints_.begin();
  for (; it != urdf_model_.joints_.end(); it++)
  {
    joint_name->push_back(it->first);
  }
}

void RobotState::clear()
{
  segments_.clear();
  map_.clear();
}

void RobotState::initSegments()
{
  // cleanning data
  clear();

  // convert URDF to KDL tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree from urdf");
  }

  // walk the tree recursively and add segments to segments_
  addChildren(kdl_tree.getRootSegment());
}

// add children to correct maps
void RobotState::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  // getting children
  const vector<KDL::SegmentMap::const_iterator> &children = segment->second.children;
  for (unsigned int i = 0; i < children.size(); i++)
  {
    // find current child
    KDL::Segment child = children[i]->second.segment;

    const string &link_name  = child.getName();
    const string &joint_name = child.getJoint().getName();

    // add segment
    segments_[link_name] = child;

    // update id tables
    map_[joint_name] = link_name;

    // continue recursively
    addChildren(children[i]);
  }
}

bool RobotState::empty(void)
{
  return segments_.empty();
}

void RobotState::getPose(const string &link_name,
                                  const double angle,
                                  KDL::Frame *pose)
{
  // get pose
  RobotStateType::const_iterator seg = segments_.find(link_name);
  if (seg != segments_.end())
    *pose = seg->second.pose(angle);
  else
    ROS_ERROR("Link name could not been found");
}

void RobotState::getPoses(PosesType &poses)
{
  poses.clear();

  // loop over all joint positions
  JointState::JointStateType::const_iterator jnt = joint_positions_.begin();
  for (; jnt != joint_positions_.end(); jnt++)
  {
    // get pose
    KDL::Frame pose;
    const string &link_name = getLinkName(jnt->first);
    getPose( link_name, jnt->second, &pose );

    // save pose
    poses[link_name] = pose;
  }
}

string RobotState::getLinkName(const string &Join_name)
{
  // get pose
  MapType::const_iterator it = map_.find(Join_name);
  if (it != map_.end())
    return it->second;
  else
    ROS_ERROR("Join name could not been found");
}

string RobotState::getJointName(const string &link_name)
{
  // get pose
  RobotStateType::const_iterator seg = segments_.find(link_name);
  if (seg != segments_.end())
    return seg->second.getJoint().getName();
  else
    ROS_ERROR("Link name could not been found");
}

}
