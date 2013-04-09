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
  // get size
  unsigned size = joint_name.size();

  // set joint names
  name_ = joint_name;

  // resize and set zeros the values
  position_.resize(size, 0);

  // create loopup table: map_
  for (unsigned i = 0; i < size; ++i)
  {
    map_[name_[i]] = i;
  }
}

bool JointState::isEmpty(void)
{
  ROS_ASSERT(name_.size() == position_.size());
  return name_.empty();
}

bool JointState::reset(void)
{
  if (!isEmpty())
  {
    // get size
    unsigned size = name_.size();

    // resize and set zeros the values
    for (unsigned i = 0; i < size; i++)
    {
      position_[i] = 0;
    }
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
    unsigned idx   = map_[joint_name[i]];
    position_[idx] = position[i];

    std::cout << std::setw(25) << std::left << name_[i];
    std::cout << " ==> pos=" << position[i] << std::endl;
  }
}

}
