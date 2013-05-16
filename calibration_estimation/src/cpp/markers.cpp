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

#include "markers.h"
#include "conversion.h"

using namespace std;
using namespace cv;

namespace calib
{

Markers::Markers() : current_id_(0)
{
  publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

  // set publish frequency
  double publish_freq;
  nh_.param("publish_frequency", publish_freq, 50.0);

  // trigger to publish fixed joints
  publish_interval_ = ros::Duration(1.0/max(publish_freq,1.0));
  timer_ = nh_.createTimer(publish_interval_, &Markers::puslish, this);
}

Markers::~Markers()
{
}

void Markers::reset()
{
  // Working around RViz bug, it doesn't delete some points of previous markers
  // if the number of checherboards are less
  for (int i = 0; i < marker_array_.markers.size(); i++)
  {
    marker_array_.markers[i].header.stamp = ros::Time();
    marker_array_.markers[i].points.clear();
  }
  puslish();
  marker_array_.markers.clear();
  current_id_ = 0;
}

void Markers::resetTime()
{
  // Working around RViz bug, it doesn't delete some points of previous markers
  // if the number of checherboards are less
  for (int i = 0; i < marker_array_.markers.size(); i++)
  {
    marker_array_.markers[i].header.stamp = ros::Time();
  }
  puslish();
}

void Markers::puslish()
{
  const ros::TimerEvent dummy_event;
  puslish(dummy_event);
}

void Markers::puslish(const ros::TimerEvent &e)
{
  publisher_.publish(marker_array_);
}

void Markers::addMarkers(const cv::Mat &board_measured_pts_3D,
                         const std::string &ns,
                         const std::string frame,
                         const cv::Scalar &color)
{
  visualization_msgs::Marker marker;
  setMarkers(current_id_, ns, frame, &marker, color);
  current_id_++;

  points2markers(board_measured_pts_3D, &marker);

  marker_array_.markers.push_back(marker);
}

void Markers::setMarkers(const int id,        // needed for MarkerArray
                const string &ns,
                const string &frame,
                visualization_msgs::Marker *marker,
                const Scalar &color, // Scalar(B,G,R)
                const double &scale)
{
  marker->id = id;
  marker->ns = ns;  // hack which allows to select the camera in RViz
  marker->header.frame_id = frame;
  marker->header.stamp = ros::Time();
//   marker->type = visualization_msgs::Marker::CUBE_LIST;
  marker->type = visualization_msgs::Marker::SPHERE_LIST;
  marker->action = visualization_msgs::Marker::ADD;

  marker->scale.x = scale;
  marker->scale.y = scale;
  marker->scale.z = scale;

  marker->color.b = color[0] / 255;
  marker->color.g = color[1] / 255;
  marker->color.r = color[2] / 255;
  marker->color.a = 1.0;
}

void Markers::points2markers(const Mat &board_measured_pts_3D,
                             visualization_msgs::Marker *marker)
{
  cv2ros(board_measured_pts_3D, &(marker->points));
}

}
