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

#include "joint_state.h"
#include "ros/assert.h"
#include "ros/debug.h"

namespace calib
{

JointState::JointState()
{
}

JointState::~JointState()
{
}

void JointState::initString(const std::vector<std::string> &joint_name)
{
  joint_positions_.clear();

  // create joint_positions_ map
  for (unsigned i = 0; i < joint_name.size(); ++i)
  {
    joint_positions_[joint_name[i]] = 0;
  }
}

bool JointState::empty(void)
{
  return joint_positions_.empty();
}

void JointState::reset(void)
{
  // reset joints to zeros
  for (JointStateType::iterator it = joint_positions_.begin();
        it != joint_positions_.end(); it++)
  {
    it->second = 0;
  }
}

bool JointState::update(const std::string &joint_name,
                        const double      &position)
{
  JointStateType::iterator it = joint_positions_.find(joint_name);
  if (it != joint_positions_.end())
  {
    it->second = position;
    return true;
  }
  else
  {
    ROS_DEBUG("Join: %s does not belong to the joint positions vector.",
              joint_name.c_str());
    return false;
  }
}

bool JointState::update(const std::vector<std::string> &joint_name,
                        const std::vector<double>      &position)
{
  // check sizes
  unsigned   size =  joint_name.size();
  ROS_ASSERT(size == position.size());

  // update position values for the given joint names
  for (unsigned i = 0; i < size; i++)
  {
    update(joint_name[i], position[i]);
//     std::cout << std::setw(25) << std::left << joint_name[i];
//     std::cout << " ==> pos=" << position[i] << std::endl;
  }
}

void JointState::getJointNames(std::vector<std::string> *joint_name) const
{
  // get joint names
  joint_name->clear();
  for (JointStateType::const_iterator it = joint_positions_.begin();
       it != joint_positions_.end(); it++)
  {
    joint_name->push_back(it->first);
  }
}

void JointState::getJointPositions(std::vector<double> *joint_position) const
{
  // get joint position
  joint_position->clear();
  for (JointStateType::const_iterator it = joint_positions_.begin();
       it != joint_positions_.end(); it++)
  {
    joint_position->push_back(it->second);
  }
}

}
