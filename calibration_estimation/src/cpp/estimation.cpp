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

/**
 * This is a app for in visualizing errors
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/calib3d/calib3d.hpp>

#include "auxiliar.h"
#include "chessboard.h"
#include "conversion.h"
#include "joint_state.h"
#include "projection.h"
#include "calibration_msgs/RobotMeasurement.h"

using namespace std;
using namespace cv;
using namespace calib;

// global variables
JointState joint_state;
ros::Publisher joint_pub;
ros::Publisher vis_pub;
robot_state_publisher::RobotStatePublisher *robot_st_publisher;
map<string, double> joint_positions;

#define NUM_COLORS 8
Scalar colors[NUM_COLORS] = {
  Scalar(255,0,0),   Scalar(0,255,0),   Scalar(0,0,255),
  Scalar(255,255,0), Scalar(255,0,255), Scalar(0,255,255),
  Scalar(255,255,255), Scalar(0,0,0)
};

/// \brief Select a color randomly
inline Scalar chooseRandomColor() { return colors[rand() % NUM_COLORS]; }
inline Scalar chooseRandomColor(int i) { return colors[i % NUM_COLORS]; }

bool initJoinStateFromParam(JointState *joint_state)
{
  vector<string> names;
  if (!getJoinNamesFromParam("robot_description", &names))
    return false;

  joint_state->initString(names);

  return true;
}

bool readRobotDescription(const string &param,
                          KDL::Tree *kdl_tree)
{
  ros::NodeHandle node;
  string robot_desc_string;
  node.param(param, robot_desc_string, string());

  if (!kdl_parser::treeFromString(robot_desc_string, *kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

//   // Get segments
//   KDL::SegmentMap robot_segments = kdl_tree->getSegments();
//
//   // Print names
//   KDL::SegmentMap::const_iterator it = robot_segments.begin();
//
//   for (unsigned i = 0; it != robot_segments.end(); it++, i++)
//   {
//     cout << it->first << endl;
//   }

  return true;
}

// ToDo: read this information from the system.yaml
void getCheckboardSize(const string &target_id, ChessBoard *cb)
{
  if (target_id == "large_cb_7x6")
  {
    cb->setSize( 7, 6, 0.108 );
    return;
  }

  if (target_id == "small_cb_4x5")
  {
    cb->setSize( 4, 5, 0.0245 );
    return;
  }
}

void print_mat(Mat tmp)
{
  cout << "size: " << tmp.rows << "x" << tmp.cols << endl;
  cout << tmp/*.t()*/ << endl;
}

// find chessboard pose using solvePnP
// TODO: move to another place
double findChessboardPose(cv::InputArray objectPoints,
                          cv::InputArray imagePoints,
                          cv::InputArray cameraMatrix,
                          cv::InputArray distCoeffs,
                          cv::OutputArray rvec,
                          cv::OutputArray tvec,
                          cv::OutputArray proj_points2D =cv::noArray())
{
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
               rvec, tvec, false, CV_ITERATIVE);

  // reprojection error
  double err = computeReprojectionErrors(objectPoints, imagePoints,
                                         cameraMatrix, distCoeffs,
                                         rvec, tvec, proj_points2D);

  return err;
}

void setMarkersParam(const int id,  // needed for MarkerArray
                     const string &ns,
                     const string &frame,
                     visualization_msgs::Marker *marker,
                     const Scalar &color =Scalar(255,255,0), // Scalar(B,G,R)
                     const double &scale =0.02)
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

void points2markers(const Mat board_measured_pts_3D,
                    visualization_msgs::Marker *marker)
{
  cv2ros(board_measured_pts_3D, &(marker->points));
}

void showMessuaremets(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement)
{
  visualization_msgs::MarkerArray marker_array;

  // generate 3D chessboard corners (board_points)
  ChessBoard cb;
  getCheckboardSize(robot_measurement->target_id, &cb);
  vector<Point3d> board_model_pts_3D;
  cb.generateCorners(&board_model_pts_3D);

  // read image points
  size_t size = robot_measurement->M_cam.size();
  for(size_t i = 0; i < size; i++)
  {
    // get camera info
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(robot_measurement->M_cam.at(i).cam_info);

    // show info
    cout << "i:" << i
         << " --  camera: " << robot_measurement->M_cam.at(i).camera_id << endl;

    // get measurement
    const vector<geometry_msgs::Point> &pts_ros = robot_measurement->M_cam.at(i).image_points;

    // convert ROS points to OpenCV
    vector<Point3d> pts;
    ros2cv(pts_ros, &pts);

    // remove last rows (this message has xyz values, with z=0 for camera)
    vector<Point2d> measured_pts_2D;
    measured_pts_2D.resize(pts.size());
    for(int j=0; j < pts.size(); j++)
    {
      measured_pts_2D[j].x = pts[j].x;
      measured_pts_2D[j].y = pts[j].y;
    }

    // find chessboard pose using solvePnP
    Mat rvec, tvec;
    vector<Point2d> expected_pts_2D;
    double err = findChessboardPose(board_model_pts_3D, measured_pts_2D,
                                    cam_model.intrinsicMatrix(),
                                    cam_model.distortionCoeffs(),
                                    rvec, tvec, expected_pts_2D);

    // board_measured_pts
    Mat board_measured_pts_3D, board_measured_pts_3D_tmp;
    project3dPoints(Mat(board_model_pts_3D), rvec, tvec, &board_measured_pts_3D);

    // TODO: Same frame!!
//     Matx31d trans(cam_model.Tx(), cam_model.Ty(), 0);
//     Mat rot;
//     Mat transformation;
//     cv::hconcat(cam_model.rotationMatrix().inv(), Mat(trans), transformation);
//     transform(board_measured_pts_3D_tmp, board_measured_pts_3D, transformation);


    // checkboard visualization
    visualization_msgs::Marker marker;
    if( robot_measurement->M_cam.at(i).camera_id == "narrow_right_rect" )
    {
      setMarkersParam(i, robot_measurement->M_cam.at(i).camera_id,
                    "narrow_stereo_r_stereo_camera_optical_frame", &marker,
                    colors[i]);
    }
    else if( robot_measurement->M_cam.at(i).camera_id == "wide_right_rect" )
    {
      setMarkersParam(i, robot_measurement->M_cam.at(i).camera_id,
                    "wide_stereo_r_stereo_camera_optical_frame", &marker,
                    colors[i]);
    }
    else
    {
      setMarkersParam(i, robot_measurement->M_cam.at(i).camera_id,
                      cam_model.tfFrame(), &marker,
                      chooseRandomColor(i));
    }

    points2markers(board_measured_pts_3D, &marker);

    marker_array.markers.push_back(marker);

    // stats
    cout << "\tcam_model.tfFrame(): "  << cam_model.tfFrame() << endl;
    cout << "\tReproj. err = "  << err << endl;
    cout << "\trvec = " << rvec << endl;
    cout << "\ttvec ="  << tvec << endl << endl;
  }

  // publish markers
  vis_pub.publish(marker_array);
}

// void robotPubliser();
void publishFixedTransforms(const ros::TimerEvent &e);

void robotMeasurementCallback(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement)
{
  // reset joints to zeros
  joint_state.reset();

  // update joints
  unsigned size = robot_measurement->M_chain.size();
  for (unsigned i = 0; i < size; i++)
  {
    joint_state.update(robot_measurement->M_chain.at(i).chain_state.name,
                       robot_measurement->M_chain.at(i).chain_state.position);
  }

  joint_positions = joint_state.getJointPositions();

  // publish moving joints
//   robotPubliser();
  ros::TimerEvent e;
  publishFixedTransforms(e);
//   robot_st_publisher->publishTransforms(joint_state.getJointPositions(),
//                                         ros::Time::now());

  // show messuaremets
  showMessuaremets(robot_measurement);

  // calculate expected values

  // publish joints
//   publishJoints(joint_pub, joint_state);
}

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment &p_segment,
              const std::string  &p_root,
              const std::string  &p_tip) : segment(p_segment), // {} //,
                                           root(p_root),
                                           tip(p_tip) { }
  KDL::Segment segment;
  std::string root, tip;
};

#include <tf_conversions/tf_kdl.h>
tf::TransformBroadcaster *tf_broadcaster_;

std::map<std::string, SegmentPair> segments_, segments_fixed_;
// std::map<std::string, KDL::Segment> segments_; //, segments_fixed_;
ros::Timer timer_;
ros::Duration publish_interval_;

// void robotPubliser()
// {
//   printf("Publishing transforms for moving joints");
//   std::vector<tf::StampedTransform> tf_transforms;
//   tf::StampedTransform tf_transform;
//   tf_transform.stamp_ = ros::Time::now();
//
//   // loop over all joints
//   for (map<string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++){
//     std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
//     if (seg != segments_.end()){
//       tf::transformKDLToTF(seg->second.segment.pose(jnt->second), tf_transform);
//       cout << "tf_transform.getOrigin(): " << tf_transform.getOrigin() << endl;
//       tf_transform.frame_id_ = seg->second.root;
//       tf_transform.child_frame_id_ = seg->second.tip;
//       tf_transforms.push_back(tf_transform);
//     }
//   }
//   tf_broadcaster_->sendTransform(tf_transforms);
// }

void publishFixedTransforms(const ros::TimerEvent &e)
{
//   ROS_DEBUG("Publishing transforms for moving joints");
  std::vector<tf::StampedTransform> tf_transforms;
  tf::StampedTransform tf_transform;
  tf_transform.stamp_ = ros::Time::now(); //+ros::Duration(0.5);  // future publish by 0.5 seconds

  // loop over all joints
  for (map<string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++){
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()){
      tf::transformKDLToTF(seg->second.segment.pose(jnt->second), tf_transform);
      cout << "tf_transform.getOrigin(): " << tf_transform.getOrigin() << endl;
      tf_transform.frame_id_ = seg->second.root;
      tf_transform.child_frame_id_ = seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
  }
  tf_broadcaster_->sendTransform(tf_transforms);
}


// add children to correct maps
// void addChildren(const KDL::SegmentMap::const_iterator segment)
// {
//   const std::string &root = segment->second.segment.getName();
//
//   const std::vector<KDL::SegmentMap::const_iterator> &children = segment->second.children;
//   for (unsigned int i=0; i < children.size(); i++)
//   {
//     const KDL::Segment &child = children[i]->second.segment;
// //     SegmentPair s( child ); //, root, child.getName() );
// //     if (child.getJoint().getType() == KDL::Joint::None)
// //     {
// //       segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
// // //       printf("Adding fixed segment from %s to %s\n", root.c_str(), child.getName().c_str());
// // //       printf("link (F): %s\t->\t", s.segment.getName().c_str());
// // //       printf("joint: %s\n", s.segment.getJoint().getName().c_str());
// //     }
// //     else
// //     {
//       segments_[child.getJoint().getName()] = child;
// //       printf("Adding moving segment from %s to %s\n", root.c_str(), child.getName().c_str());
//       printf("link (M): %s\t->\t", child.getName().c_str());
//       printf("joint: %s\n", child.getJoint().getName().c_str());
// //     }
// //     printf("link (M): %s\t->\t", child.getJoint().getName().c_str() );
// //     printf("link (M): %s\n", child.getName().c_str() );
//
//     addChildren(children[i]);
//   }
// }

// std::map<std::string, KDL::Segment> segments_; //, segments_fixed_;

// add children to correct maps
// void addChildren(const KDL::SegmentMap::const_iterator segment)
// {
//   const std::string &root = segment->second.segment.getName();
//
//   const std::vector<KDL::SegmentMap::const_iterator> &children = segment->second.children;
//   for (unsigned int i=0; i < children.size(); i++)
//   {
//     const KDL::Segment &child = children[i]->second.segment;
//     segments_[child.getJoint().getName()] = child;
//
//     printf("link (M): %s\t->\t", child.getName().c_str());
//     printf("joint: %s\n", child.getJoint().getName().c_str());
//
//     addChildren(children[i]);
//   }
// }




void addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = segment->second.segment.getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;
  for (unsigned int i=0; i<children.size(); i++){
    const KDL::Segment& child = children[i]->second.segment;
    SegmentPair s(children[i]->second.segment, root, child.getName());
//     if (child.getJoint().getType() == KDL::Joint::None){
//       segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
//       ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
//     }
//     else{
      segments_.insert(make_pair(child.getJoint().getName(), s));
//       printf("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
//     }
    addChildren(children[i]);
  }
}



void getJoinNamesFromKDL(const KDL::Tree &tree)
{
  // walk the tree and add segments to segments_
  addChildren(tree.getRootSegment());
}


// void readRobotDescriptionFromTopic(std_msgs::StringConstPtr robot_description, KDL::Tree *kdl_tree)
// {
//   /*KDL::Tree kdl_tree*/;
//
// //   string robot_description;
//
//   if (!kdl_parser::treeFromString(robot_description->data, *kdl_tree))
//   {
//     ROS_ERROR("Failed to construct kdl tree");
// //     return false;
//   }
//
//   // Get segments
//   KDL::SegmentMap robot_segments = kdl_tree->getSegments();
//
//   // Print names
//   KDL::SegmentMap::const_iterator it = robot_segments.begin();
//
//   for (unsigned i = 0; it != robot_segments.end(); it++, i++)
//   {
//     cout << it->first << endl;
//   }
//
//   printf("NAMES!!!\n");
//   getJoinNamesFromKDL(kdl_tree);
//
// //   return true;
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualization");

  if (!initJoinStateFromParam(&joint_state))
  {
    return 0;
  }

  // read robot description and create robot_state_publisher
  KDL::Tree kdl_tree;
  readRobotDescription("robot_description", &kdl_tree);
  if (!robot_st_publisher)
    robot_st_publisher = new robot_state_publisher::RobotStatePublisher(kdl_tree);

  getJoinNamesFromKDL(kdl_tree);
  cout << "segments_.size(): " << segments_.size() << endl;
//   cout << "segments_fixed_.size(): " << segments_fixed_.size() << endl;

  vector<string> joint_names;
  joint_state.getJointNames(&joint_names);
  cout << "joint_names.size(): " << joint_names.size() << endl;
//   cout << joint_names;
//   return 0;


  // create node
  ros::NodeHandle n; //("calib");

  tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster_ = &tf_broadcaster;

  // subscriber
//   ros::Subscriber sub_robot_description = n.subscribe<std_msgs::String>("robot_description", 1,
//                                           boost::bind(readRobotDescriptionFromTopic, _1, &kdl_tree ));


  // trigger to publish fixed joints
  publish_interval_ = ros::Duration(1.0/max(50.0,1.0));
  timer_ = n.createTimer(publish_interval_, &publishFixedTransforms);

  // subscriber
  ros::Subscriber subs_robot_measurement = n.subscribe("robot_measurement", 1,
                                                       robotMeasurementCallback);
  // publisher
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  // visualization marker publisher
  vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

  ros::spin();

  delete robot_st_publisher;
  return 0;
}

