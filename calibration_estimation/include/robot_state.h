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

#include <map>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>

namespace calib
{

class RobotState
{
public:
  typedef std::map<std::string, KDL::Segment> RobotStateType;
  typedef std::map<std::string, std::string>  MapType;

  RobotState();
  ~RobotState();

  /// \brief Load from KDL tree
  void initFromTree(const KDL::Tree &tree);

  /// \brief Check if it is empty (valid)
  bool empty();

  /// \brief Reset postion vector to zeros
  bool reset();

  void getFK(std::vector<KDL::Frame> &frames);

protected:
  /// \brief Add children. It creates the 'segments_' map
  void addChildren(const KDL::SegmentMap::const_iterator segment);

private:
  RobotStateType segments_;   // (map) link_name  -> KDL::Segment
  MapType        map_;        // (map) joint_name -> link_name
  JointState     joint_state; //       joint_name -> angles
};

}

#endif // ROBOT_STATE_H

