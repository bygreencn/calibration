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


#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include <vector>
#include <string>
#include <map>

namespace calib
{

class JointState
{
public:
  typedef std::map<std::string, double> JointStateType;

  JointState();
  ~JointState();

  /// \brief Load joint names from vector of string
  void initString(const std::vector<std::string> &joint_name);

  /// \brief Check if it is empty (valid)
  bool empty();

  /// \brief Reset postion vector to zeros
  bool reset();

  /// \brief Update one joint postions
  bool update(const std::string &joint_name,
              const double      &position);

  /// \brief Update joint postions (it can be an incomplete subset)
  bool update(const std::vector<std::string> &joint_name,
              const std::vector<double>      &position);

  /// \brief get Joint Names vector
  void getJointNames(std::vector<std::string> *joint_name) const;

  /// \brief get Joint Positions vector
  void getJointPositions(std::vector<double> *joint_position) const;

  const JointStateType &getJointPositions() const { return joint_positions_; }

private:
  JointStateType joint_positions_;
};

}

#endif // JOINT_STATE_H

