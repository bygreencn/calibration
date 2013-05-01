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


#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "joint_state.h"

#include <urdf/model.h>
#include <map>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

namespace calib
{

class RobotState : public JointState
{
public:
  typedef std::map<std::string, KDL::Frame> PosesType;

  RobotState();
  ~RobotState();

  /// \brief Load from urdf Model
  void initFromURDF(const urdf::Model &model);

  /// \brief Get Joint Names vector
  void getJointNames(std::vector<std::string> *joint_name) const;

  /// \brief Check if it is empty (valid)
  bool empty();

  /// \brief get pose
  void getPose(const std::string &link_name,
               const double angle,
               KDL::Frame *pose) const;

  /// \brief get poses
  void getPoses(PosesType *poses) const;

  /// \brief Get root of the link
  std::string getRoot(const std::string &link_name) const;

  /// \brief get LinkName from JointName
  std::string getLinkName(const std::string &Join_name) const;

  /// \brief get JointName from LinkName
  std::string getJointName(const std::string &link_name) const;

  /// \brief get JointType from LinkName ('KDL::Joint::None' means fixed joint)
  KDL::Joint::JointType getJointType(const std::string &link_name) const;

protected:
  /// \brief Clear internal structures
  void clear();

  /// \brief Update KDL tree from URDF
  void updateTree();

  /// \brief KDL Joints has an ID (q_nr)
  unsigned getJointID(const std::string &link_name) const;

  /// \brief Generate JntArray (KDL type) from JointState::join_state
  void generateJntArray(KDL::JntArray *jnt_array);

  /// \brief Get SegmentMap from KDL tree
  const KDL::SegmentMap &segments() const { return kdl_tree_->getSegments(); }

protected:
  urdf::Model  urdf_model_;  // URDF model
  KDL::Tree   *kdl_tree_;    // KDL tree (data from urdf but used for kinematic)
};

}

#endif // ROBOT_STATE_H

