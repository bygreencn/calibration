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

#ifndef CONVERSION_H
#define CONVERSION_H

#include <geometry_msgs/Point.h>
#include <opencv2/core/core.hpp>
#include <kdl/frames.hpp>

namespace calib
{

/// \brief Conversion functions: convert ROS to OpenCV points
void ros2cv(const geometry_msgs::Point &point_ros, cv::Point3d *point_cv);
void ros2cv(const std::vector<geometry_msgs::Point> &pts_ros, std::vector<cv::Point3d> *pts_cv);
void ros2cv(const std::vector<geometry_msgs::Point> &pts_ros, cv::Mat_<double> *pts_cv);

/// \brief Conversion functions: convert OpenCV to ROS points
void cv2ros(const cv::Mat &pt_cv, std::vector<geometry_msgs::Point> *pt_ros);

/// KDL <-> OpenCV
void kdl2cv(const KDL::Frame    &frame,       cv::OutputArray R, cv::OutputArray t);
void kdl2cv(const KDL::Rotation &rotation,    cv::OutputArray R);
void kdl2cv(const KDL::Vector   &translation, cv::OutputArray t);
void cv2kdl(const cv::InputArray R, KDL::Rotation *rotation);
void cv2kdl(const cv::InputArray t, KDL::Vector   *translation);


/// Serialization & Deserialization
// Point3d <-> double[3]
void   serialize(const cv::Point3d &in, double out[3]);
void deserialize(const double   out[3], cv::Point3d *in);

// Point2d <-> double[2]
void   serialize(const cv::Point2d &in, double out[2]);
void deserialize(const double   out[2], cv::Point2d *in);

// vector<Point3d> <-> vector<double*>
void   serialize(const std::vector<cv::Point3d> &in, std::vector<double *> *out);
void deserialize(const std::vector<double *>   &out, std::vector<cv::Point3d> *in);

// KDL::Rotation <-> double[4] (quaternions)
void   serialize(const KDL::Rotation   &rotation, double camera_rotation[4]);
void deserialize(const double camera_rotation[4], KDL::Rotation *rotation);

// KDL::Vector <-> double[3]
void   serialize(const KDL::Vector     &translation, double camera_translation[3]);
void deserialize(const double camera_translation[3], KDL::Vector *translation);


void deserialize(const double    camera_rotation[4], cv::Matx33d *R);
void deserialize(const double camera_translation[3], cv::Vec3d   *tvec);

// intrinsicMatrix <-> double[4]
void   serializeIntrinsics(const cv::Mat_<double> &intrinsicMatrix, double K[4]);
void deserializeIntrinsics(const double K[4], cv::Mat_<double> *intrinsicMatrix);

}

#endif // CONVERSION_H
