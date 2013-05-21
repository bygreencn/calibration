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


#ifndef MARKERS_H
#define MARKERS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core/core.hpp>

namespace calib
{

/// \brief Select a color randomly
cv::Scalar chooseRandomColor();
cv::Scalar chooseColor(int i);

class Markers
{
public:
  Markers();
  ~Markers();

  /// \brief Reset markers (clean the marker_array)
  void reset();
  void resetTime();

  /// \brief Add markers to publish
  void addMarkers(const cv::Mat &board_measured_pts_3D,
                  const std::string &ns,
                  const std::string &frame,
                  const cv::Scalar &color);

  /// \brief Publish markers
  void puslish();
  void puslish(const ros::TimerEvent &e);

protected:
  /// \brief Set markers properties
  void setMarkers(const int id,  // needed for MarkerArray
                  const std::string &ns,
                  const std::string &frame,
                  visualization_msgs::Marker *marker,
                  const cv::Scalar &color = cv::Scalar(255, 255, 0), // Scalar(B,G,R)
                  const double &scale = 0.02);

  /// \brief convert cv points to markers
  void points2markers(const cv::Mat &board_measured_pts_3D,
                    visualization_msgs::Marker *marker);

private:
  ros::NodeHandle nh_;
  ros::Publisher  publisher_;

  ros::Timer      timer_;
  ros::Duration   publish_interval_;

  visualization_msgs::MarkerArray marker_array_;

  unsigned current_id_; // using same id override previous markers
};


}

#endif // MARKERS_H

