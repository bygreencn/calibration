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
#include <kdl_parser/kdl_parser.hpp>

using namespace std;

namespace calib
{

RobotState::RobotState() : kdl_tree_(0), fk_solver(0)
{
}

RobotState::~RobotState()
{
  deletePtrs();
}

void RobotState::initFromURDF(const urdf::Model &model)
{
  // copy urdf model
  urdf_model_ = model;

  // create internal structures
  updateTree();

  // get joint names
  vector<string> joint_names;
  getJointNames(&joint_names);

  // init joint_state (vector of angles joints)
  JointState::initString(joint_names);
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
  urdf_model_.clear();
  deletePtrs();
}

void RobotState::deletePtrs()
{
  // delete current KDL tree
  if (kdl_tree_)
    delete kdl_tree_;

  // delete current fk_solver
  if (fk_solver)
    delete fk_solver;
}

void RobotState::updateTree()
{
  // delete any previous pointers, if exist
  deletePtrs();

  // convert URDF to KDL tree
  kdl_tree_ = new KDL::Tree();
  if (!kdl_parser::treeFromUrdfModel(urdf_model_, *kdl_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree from urdf");
  }

  // create TreeFkSolverPos_recursive from KDL tree
  fk_solver = new KDL::TreeFkSolverPos_recursive(*kdl_tree_);
}

bool RobotState::empty(void)
{
  return segments().empty();
}

bool RobotState::getFK(const std::string &link_name, KDL::Frame *pose)
{
  // generate Joint Array
  KDL::JntArray q_in;
  generateJntArray(&q_in);

  // forward kinematic (recursive)
  int result = fk_solver->JntToCart(q_in, *pose, link_name);

  if (result == -1)
    printf("q_in.rows() != tree.getNrOfJoints()");

  if (result == -2)
    printf("Link name could not been found");

  return result >= 0;
}

void RobotState::getPose(const string &link_name,
                         const double angle,
                         KDL::Frame *pose) const
{
  // get pose
  KDL::SegmentMap::const_iterator seg = segments().find(link_name);
  if (seg != segments().end())
    *pose = seg->second.segment.pose(angle);
  else
    ROS_ERROR("Link name could not been found");
}

void RobotState::getPoses(PosesType *poses) const
{
  poses->clear();

  // loop over all joint positions
  JointState::JointStateType::const_iterator jnt = joint_positions_.begin();
  for (; jnt != joint_positions_.end(); jnt++)
  {
    // get pose
    KDL::Frame pose;
    const string &link_name = getLinkName(jnt->first);
    getPose(link_name, jnt->second, &pose);

    // save pose
    (*poses)[link_name] = pose;
  }
}

string RobotState::getRoot(const string &link_name) const
{
  const urdf::Link *link = urdf_model_.getLink(link_name).get();
  if (!link)
  {
    ROS_ERROR("Link name could not been found");
    return string("");
  }

  const urdf::Link *parent = link->getParent().get();
  if (!parent)
  {
    ROS_ERROR("Parent link name could not been found");
    return string("");
  }

  return parent->name;
}

string RobotState::getLinkName(const string &Join_name) const
{
  // get pose
  map <string, boost::shared_ptr<urdf::Joint > >::const_iterator it = urdf_model_.joints_.find(Join_name);
  if (it != urdf_model_.joints_.end())
    return it->second->child_link_name;
  else
    ROS_ERROR("Join name could not been found");
}

string RobotState::getJointName(const string &link_name) const
{
  // get pose
  KDL::SegmentMap::const_iterator seg = segments().find(link_name);
  if (seg != segments().end())
    return seg->second.segment.getJoint().getName();
  else
  {
    ROS_ERROR("Link name could not been found");
    return string("");
  }
}

KDL::Joint::JointType RobotState::getJointType(const std::string &join_name) const
{
  KDL::SegmentMap::const_iterator seg = segments().find(getLinkName(join_name));
  if (seg != segments().end())
    return seg->second.segment.getJoint().getType();
  else
  {
    ROS_ERROR("Link name could not been found");
    return KDL::Joint::None;
  }
}

int RobotState::getJointID(const std::string &join_name) const
{
  KDL::SegmentMap::const_iterator seg = segments().find(getLinkName(join_name));
  if (seg != kdl_tree_->getSegments().end())
  {
    return seg->second.q_nr;
  }
  else
  {
    printf("Link name could not been found\n");
    return -1;
  }
}

void RobotState::generateJntArray(KDL::JntArray *jnt_array)
{
  jnt_array->resize(kdl_tree_->getNrOfJoints());
  KDL::SetToZero(*jnt_array);

  JointStateType::const_iterator it = joint_positions_.begin();
  for (; it != joint_positions_.end(); it++)
  {
    jnt_array->data(getJointID(it->first)) = it->second;
  }
}

}
