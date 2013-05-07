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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/calib3d/calib3d.hpp>

#include "auxiliar.h"
#include "chessboard.h"
#include "conversion.h"
#include "joint_state.h"
#include "projection.h"
#include "robot_state.h"
#include "robot_state_publisher.h"
#include "calibration_msgs/RobotMeasurement.h"

using namespace std;
using namespace cv;
using namespace calib;

// global variables
RobotStatePublisher *robot_state;
ros::Publisher vis_pub;

#define NUM_COLORS 8
Scalar colors[NUM_COLORS] = {
  Scalar(255,0,0),   Scalar(0,255,0),   Scalar(0,0,255),
  Scalar(255,255,0), Scalar(255,0,255), Scalar(0,255,255),
  Scalar(255,255,255), Scalar(0,0,0)
};

/// \brief Select a color randomly
inline Scalar chooseRandomColor() { return colors[rand() % NUM_COLORS]; }
inline Scalar chooseColor(int i)  { return colors[i % NUM_COLORS]; }

// TODO: read this information from the system.yaml
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

void setMarkers(const int id,  // needed for MarkerArray
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

void transform3DPoints(const cv::Mat &points,
                       const KDL::Frame &frame,
                       cv::Mat *modif_points )
{
  cv::Mat R, t;
  kdl2cv(frame, R, t);

  Mat transformation;
  cv::hconcat(R, t, transformation);

  transform(points, *modif_points, transformation);
}

void transform3DPoints(const cv::Mat &points,
                       const string &frame,
                       cv::Mat *modif_points )
{
  KDL::Frame pose;
  robot_state->getFK(frame, &pose);
  transform3DPoints(points, pose, modif_points);
}

void transform3DPoints(const cv::Mat &points,
                       const string &frame1,
                       const string &frame2,
                       cv::Mat *modif_points )
{
  KDL::Frame pose1, pose2;
  robot_state->getFK(frame1, &pose1);
  robot_state->getFK(frame2, &pose2);
  transform3DPoints(points, pose2 * pose1.Inverse(), modif_points);
}

void showMessuaremets(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement)
{
  // Working around RViz bug, it doesn't delete some points of previous markers
  // if the number of checherboards are less
  static visualization_msgs::MarkerArray marker_array;
  for (int i=0; i < marker_array.markers.size(); i++)
  {
    marker_array.markers[i].header.stamp = ros::Time();
    marker_array.markers[i].points.clear();
  }
  vis_pub.publish(marker_array);
  marker_array.markers.clear();

  // generate 3D chessboard corners (board_points)
  ChessBoard cb;
  getCheckboardSize(robot_measurement->target_id, &cb);
  vector<Point3d> board_model_pts_3D;
  cb.generateCorners(&board_model_pts_3D);

  // read image points
  size_t size = robot_measurement->M_cam.size();
  for (size_t i = 0; i < size; i++)
  {
    // get camera info
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(robot_measurement->M_cam.at(i).cam_info);

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
    Mat D; // empty for rectified cameras
    // Mat D = cam_model.distortionCoeffs();  // for non-rectified cameras
    double err = findChessboardPose(board_model_pts_3D, measured_pts_2D,
                                    cam_model.intrinsicMatrix(),
                                    D, rvec, tvec, expected_pts_2D);

    // board_measured_pts
    Mat board_measured_pts_3D, board_measured_pts_3D_tmp;
    transform3DPoints(Mat(board_model_pts_3D), rvec, tvec, &board_measured_pts_3D);

    // TODO: Same frame!!
//     Matx31d trans(cam_model.Tx(), cam_model.Ty(), 0);
//     Mat rot;
//     Mat transformation;
//     cv::hconcat(cam_model.rotationMatrix().inv(), Mat(trans), transformation);
//     transform(board_measured_pts_3D_tmp, board_measured_pts_3D, transformation);


    // checkboard visualization
    // TODO: some frame are hard code here. Is it a possible error in the bag?
    visualization_msgs::Marker marker;
    if( robot_measurement->M_cam.at(i).camera_id == "narrow_right_rect" )
    {
      setMarkers(i, robot_measurement->M_cam.at(i).camera_id,
                    "narrow_stereo_r_stereo_camera_optical_frame", &marker,
                    colors[i]);
    }
    else if( robot_measurement->M_cam.at(i).camera_id == "wide_right_rect" )
    {
      setMarkers(i, robot_measurement->M_cam.at(i).camera_id,
                    "wide_stereo_r_stereo_camera_optical_frame", &marker,
                    colors[i]);
    }
    else
    {
      setMarkers(i, robot_measurement->M_cam.at(i).camera_id,
                      cam_model.tfFrame(), &marker,
                      chooseColor(i));
    }

    points2markers(board_measured_pts_3D, &marker);
    marker_array.markers.push_back(marker);


    //! choose one (example). TODO: delete this!
    string frame, frame2, f;
    visualization_msgs::Marker m, m2;
    if (i == 0)
    {
      frame  = "narrow_stereo_optical_frame"; //cam_model.tfFrame();
      frame2 = "wide_stereo_optical_frame";
      f = "base_footprint";
//       frame2 = f;

      Mat new_pts_3D, new_pts_3D_2;
      transform3DPoints(Mat(board_model_pts_3D), frame, &new_pts_3D);

      setMarkers(i, "new",
                    f, &m,
                    colors[i+1]);
      points2markers(new_pts_3D, &m);
      marker_array.markers.push_back(m);


      cv::Mat rvec, tvec;
      transform3DPoints(new_pts_3D, frame, frame2, &new_pts_3D_2);

      setMarkers(i, "new2",
                    f, &m2,
                    colors[i+2]);
      points2markers(new_pts_3D_2, &m2);
      marker_array.markers.push_back(m2);
    }


    // show info
    cout << "i:" << i
         << " --  camera: " << robot_measurement->M_cam.at(i).camera_id << endl;
    cout << "\tcam_model.tfFrame(): "  << cam_model.tfFrame() << endl;
    cout << "\tReproj. err = "  << err << endl;
    cout << "\trvec = " << rvec << endl;
    cout << "\ttvec ="  << tvec << endl << endl;
  }

  // publish markers
  vis_pub.publish(marker_array);
}

void robotMeasurementCallback(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement)
{
  // reset joints to zeros
  robot_state->reset();

  // update joints
  unsigned size = robot_measurement->M_chain.size();
  for (unsigned i = 0; i < size; i++)
  {
    robot_state->update(robot_measurement->M_chain.at(i).chain_state.name,
                        robot_measurement->M_chain.at(i).chain_state.position);
  }

  // show messuaremets
  showMessuaremets(robot_measurement);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimation");

  // read urdf model from ROS param
  urdf::Model model;
  if (!model.initParam("robot_description"))
    return EXIT_FAILURE;

  // save urdf to file
//   TiXmlDocument* output = urdf::exportURDF( model );
//   if (!output)
//     ROS_ERROR("Failed to save urdf file\n");
//   else
//     output->SaveFile("urdf_calibrated.xml");


  // create a KDL tree from Urdf Model
//   KDL::Tree kdl_tree;
//   if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
//   {
//     ROS_ERROR("Failed to construct kdl tree from urdf");
//     return EXIT_FAILURE;
//   }

//   const urdf::Link *root = model.getRoot().get();
//   cout << "Link name: " << root->name << endl;// << " - Joint name: " << root->parent_joint.get()->name << endl;
//
//   PRINT(root->child_links.size(),"\n");
//
//   const urdf::Link *base_link = model.links_["base_link"].get();
//   cout << "Link name: " << base_link->name << " - Joint name: " << base_link->parent_joint.get()->name << endl;
//   PRINT(base_link->child_links.size(),"\n");
//   for (int i=0; i<base_link->child_links.size(); i++)
//   {
// //     PRINT(base_link->child_links.at(i).get()->name);
//     PRINT(base_link->child_links.at(i).get()->parent_joint->name,": ")
//        << base_link->child_joints.at(i).get()->name << "\n";
// //     ->getParent().get()->parent_joint.get()->name);
//   }

  // robot state publisher
//   if (!robot_st_publisher)
//     robot_st_publisher = new robot_state_publisher::RobotStatePublisher(kdl_tree);

  // robot init
  robot_state = new RobotStatePublisher();
  robot_state->initFromURDF(model);

  // create node
  ros::NodeHandle n; //("calib");

  // subscriber
  ros::Subscriber subs_robot_measurement = n.subscribe("robot_measurement", 1,
                                                       robotMeasurementCallback);

  // visualization marker publisher
  vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

  ros::spin();

//   delete robot_st_publisher;
  return 0;
}
