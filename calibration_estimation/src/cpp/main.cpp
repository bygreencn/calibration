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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <std_msgs/String.h>
#include <kdl_parser/kdl_parser.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <assert.h>

#include "auxiliar.h"
#include "chessboard.h"
#include "conversion.h"
#include "joint_state.h"
#include "markers.h"
#include "projection.h"
#include "robot_state.h"
#include "robot_state_publisher.h"
#include "calibration_msgs/RobotMeasurement.h"

#include "cost_functions.h"
#include "optimization.h"

using namespace std;
using namespace cv;
using namespace calib;

// global variables
RobotState *robot_state;
Markers    *visual_markers;
Data       *data;

/// \brief transform 3D using the Rotation and Translation defined in frame
void transform3DPoints(const cv::Mat &points,
                       const string &frame,
                       cv::Mat *modif_points )
{
  KDL::Frame pose;
  robot_state->getFK(frame, &pose);
  transform3DPoints(points, pose, modif_points);
}

/// \brief transform 3D from frame1 to frame2
void transform3DPoints(const cv::Mat &points,
                       const string &frame1,
                       const string &frame2,
                       cv::Mat *modif_points )
{
  KDL::Frame pose1, pose2;
  robot_state->getFK(frame1, &pose1);
  robot_state->getFK(frame2, &pose2);
  transform3DPoints(points, pose2.Inverse() * pose1, modif_points);
}

void showMessuaremets(const calibration_msgs::RobotMeasurement::ConstPtr &robot_measurement)
{
  visual_markers->reset();

  // generate 3D chessboard corners (board_points)
  ChessBoard cb;
  getCheckboardSize(robot_measurement->target_id, &cb);
  vector<Point3d> board_model_pts_3D;
  cb.generateCorners(&board_model_pts_3D);



  ceres::Problem problem;

  // one camera 3D points (used solvePnP for now, but maybe
  // multiview triangulation can be used as initilization)
  vector<double *> param_point_3D;
  vector<double *> param_camera_rot;
  vector<double *> param_camera_trans;

  vector<KDL::Frame> cb2camera, corrected, pose_rel, pose_father;
  Mat rvec0, tvec0;

  KDL::Frame T, T0;
  vector<string> frame_name;
  Mat board_pts_frame0, board_pts_frame1;

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

    KDL::Frame current_cb2camera;
    cv2kdl(rvec, tvec, &current_cb2camera);
    cb2camera.push_back(current_cb2camera);

    // board_measured_pts
    Mat board_measured_pts_3D, board_measured_pts_3D_tmp;
    transform3DPoints(Mat(board_model_pts_3D), rvec, tvec, &board_measured_pts_3D);
    if (i == 0)
    {
      board_measured_pts_3D.copyTo(board_pts_frame0);
      rvec.copyTo(rvec0);
      tvec.copyTo(tvec0);
      corrected.push_back(current_cb2camera);
    }
    else
    {
      corrected.push_back( corrected.at(0) * current_cb2camera.Inverse() );
    }

//     if(i==1)
//     {
//       // reprojection error
//       Mat D;
//       Mat R = Mat::eye(3, 3, CV_64F);
//       Mat rvec;
//       Rodrigues(R,rvec);
//       Mat tvec = Mat::zeros(3,1,CV_64F);
//       double err = computeReprojectionErrors(board_pts_frame1, measured_pts_2D,
//                                              cam_model.intrinsicMatrix(),
//                                              D, rvec, tvec, expected_pts_2D);
//       cout << "board_pts_frame1 error = " << err << endl;
//
// //       PRINT(measured_pts_2D)
// //       PRINT(expected_pts_2D)
//     }


    // Calculate error
    double current_error=0;
    for (int k = 0; k < board_model_pts_3D.size(); k++)
    {
      double _point3D[3];
      serialize(board_model_pts_3D[k], _point3D);

      double _point2D[2];
      serialize(measured_pts_2D[k], _point2D);

      double _camera_rotation[4];
      KDL::Rotation _r;
      cv2kdl(rvec, &_r);
      serialize(_r,_camera_rotation);

      KDL::Vector _t;
      cv2kdl(tvec, &_t);
      double _camera_translation[3];
      serialize(_t,_camera_translation);

      Mat_<double> intrinsicMatrix = cam_model.intrinsicMatrix();
      double _proj_point2D[2];
      double residuals[2];
      calc_residuals(_point2D[0], _point2D[1],
                     intrinsicMatrix(0, 0),
                     intrinsicMatrix(1, 1),
                     intrinsicMatrix(0, 2),
                     intrinsicMatrix(1, 2),
                     _camera_rotation,
                     _camera_translation,
                     _point3D,
                     residuals);

      double error = calc_norm(residuals);

      current_error += error;
    }
    PRINT(current_error);
    cout << "Aveg. error: " << current_error/board_model_pts_3D.size() << endl;


    // TODO: Same frame!!
//     Matx31d trans(cam_model.Tx(), cam_model.Ty(), 0);
//     Mat rot;
//     Mat transformation;
//     cv::hconcat(cam_model.rotationMatrix().inv(), Mat(trans), transformation);
//     transform(board_measured_pts_3D_tmp, board_measured_pts_3D, transformation);


    // checkboard visualization
    string current_frame;

    // TODO: some frame are hard code here. Is it a possible error in the bag?
    if( robot_measurement->M_cam.at(i).camera_id == "narrow_left_rect" )
      current_frame = "narrow_stereo_l_stereo_camera_optical_frame";
    else if( robot_measurement->M_cam.at(i).camera_id == "narrow_right_rect" )
      current_frame = "narrow_stereo_r_stereo_camera_optical_frame";
    else if( robot_measurement->M_cam.at(i).camera_id == "wide_left_rect" )
      current_frame = "wide_stereo_l_stereo_camera_optical_frame";
    else if( robot_measurement->M_cam.at(i).camera_id == "wide_right_rect" )
      current_frame = "wide_stereo_r_stereo_camera_optical_frame";
    else if( cam_model.tfFrame() == "/head_mount_kinect_rgb_optical_frame" )
      current_frame = "head_mount_kinect_rgb_optical_frame";
    else
      current_frame = cam_model.tfFrame();

    // get relative pose (camera to its father)
    KDL::Frame pose;
    robot_state->getRelativePose(current_frame, &pose);
    pose_rel.push_back(pose);
    const string link_root = robot_state->getLinkRoot(current_frame);

    // get father pose (father to tree root)
    KDL::Frame pose_f;
    robot_state->getFK(link_root, &pose_f);
    pose_father.push_back(pose_f);
//     pose = pose_f * pose;

    frame_name.push_back(current_frame);
    Mat modif_points;
    transform3DPoints(board_measured_pts_3D,
                      current_frame, "base_footprint",
                      &modif_points);
    visual_markers->addMarkers(modif_points,
                               robot_measurement->M_cam.at(i).camera_id,
                               "base_footprint",
                               chooseColor(i));

//     points2markers(board_measured_pts_3D, &marker);
//     marker_array.markers.push_back(marker);


//     //! choose one (example). TODO: delete this!
//     string frame0, frame1, f;
//     visualization_msgs::Marker m, m2;
//     if (i == 1)
//     {
//       frame0  = "narrow_stereo_optical_frame"; //cam_model.tfFrame();
//       frame1 = "wide_stereo_optical_frame";
//       f = "base_footprint";
//
//       // T_0 (camera '0' KDL::Frame to robot)
//       robot_state->getFK(frame0, &T0);
// //       transform3DPoints(board_pts_frame0, T0, &new_pts_3D);
//
//       // T
//       robot_state->getFK(frame1, &T);
//       KDL::Frame R, R_prime, E;
//       R = T.Inverse() * T0;
//       transform3DPoints(board_pts_frame0, R, &board_pts_frame1); // in the frame1
//
//       Mat board_pts_frame1_using_correction;
//       R_prime = corrected.at(1) * corrected.at(0).Inverse();   // corrected fram1
//       E = R_prime * R.Inverse();                               // frame error
//       transform3DPoints(board_pts_frame0,
//                         E.Inverse() * R_prime, &board_pts_frame1_using_correction);
//
//       setMarkers(i, "new",
//                     frame1, &m,
//                     colors[i+1]);
//       points2markers(board_pts_frame1_using_correction, &m);
//       marker_array.markers.push_back(m);
//
// //       transform3DPoints(new_pts_3D, frame, frame2, &new_pts_3D_2);
//
//       setMarkers(i, "new2",
//                     frame1, &m2,
//                     colors[i+2]);
//       points2markers(board_pts_frame1, &m2);
//       marker_array.markers.push_back(m2);
//     }


    //! Optimization
    if (i == 0)
    {
      // generate 3D points
      serialize(board_pts_frame0, &param_point_3D);
      vector<Point3d> test;
      deserialize(param_point_3D, &test);

      // T_0 (camera '0' KDL::Frame to robot)
      robot_state->getFK(current_frame, &T0);
//       KDL::Frame current_position = T0.Inverse() * T0;  // identity
    }

    robot_state->getFK(current_frame, &T);
    KDL::Frame current_position = (pose_father[i]*pose_rel[i]).Inverse() * T0;

    // generate cameras
    double *camera_rot = new double[4];
    serialize(current_position.M, camera_rot);
    param_camera_rot.push_back(camera_rot);

    double *camera_trans = new double[3];
    serialize(current_position.p, camera_trans);
    param_camera_trans.push_back(camera_trans);

    Matx33d intrinsicMatrix = cam_model.intrinsicMatrix();
    // feed optimazer with data
    assert(measured_pts_2D.size() == board_model_pts_3D.size());
    for (int j = 0; j < measured_pts_2D.size(); j++)
    {
      ceres::CostFunction *cost_function =
        ReprojectionErrorWithQuaternions2::Create(measured_pts_2D[j].x,
                                                  measured_pts_2D[j].y,
                                                  intrinsicMatrix(0,0),
                                                  intrinsicMatrix(1,1),
                                                  intrinsicMatrix(0,2),
                                                  intrinsicMatrix(1,2),
                                                  param_point_3D[j]
                                                  );

      problem.AddResidualBlock(cost_function,
                              NULL,                       // squared loss
                              param_camera_rot[i],      // camera_rot i
                              param_camera_trans[i]    // camera_trans i
                              );         // point j
    }

    if (i == 0)
    {
      problem.SetParameterBlockConstant(camera_rot);
      problem.SetParameterBlockConstant(camera_trans);
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
  visual_markers->puslish();


  for (int i = 0; i < robot_measurement->M_cam.size(); i++)
  {
    print_array(param_camera_rot[i], 4,   "param_camera[i]:");
    print_array(param_camera_trans[i], 3, "               :");
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
//   options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  cout << "\n";
  for (int i = 0; i < robot_measurement->M_cam.size(); i++)
  {
    print_array(param_camera_rot[i],   4, "param_camera[i]:");
    print_array(param_camera_trans[i], 3, "               :");
  }

  // i=1 (update all cameras except the first one)
  for (int i = 1; i < robot_measurement->M_cam.size(); i++)
  {
    KDL::Frame frame;
    double *camera_rot = param_camera_rot[i];
    double *camera_trans = param_camera_trans[i];
    frame.M.Quaternion(camera_rot[0],
                       camera_rot[1],
                       camera_rot[2],
                       camera_rot[3]);

    frame.p.data[0] = camera_trans[0];
    frame.p.data[1] = camera_trans[1];
    frame.p.data[2] = camera_trans[2];


    KDL::Frame current_position = pose_father[i].Inverse() * T0 * frame.Inverse();
    urdf::Pose pose;
    kdl2urdf(current_position, &pose);
    robot_state->setUrdfPose(frame_name[i], pose);
  }

  sleep(1);
  robot_state->updateTree();

  visual_markers->resetTime();




// //   int i=1;
//   for( int i=1; i<corrected.size(); i++)
//   {
//     KDL::Frame R = (pose_father[i] * pose_rel[i]).Inverse() * T0;
//
//     KDL::Frame new_rel;
//     new_rel = pose_father[i].Inverse() * T0 * corrected[i];// R.Inverse();
//
//     urdf::Pose pose = robot_state->getUrdfPose(frame_name.at(i));
//     KDL::Frame fr = new_rel;
//     double x,y,z,w;
//     fr.M.GetQuaternion(x,y,z,w);
//     pose.rotation.x = x;    /// pose es relativo to its father!!
//     pose.rotation.y = y;
//     pose.rotation.z = z;
//     pose.rotation.w = w;
//     pose.position.x = fr.p.x();
//     pose.position.y = fr.p.y();
//     pose.position.z = fr.p.z();
//     robot_state->setUrdfPose(frame_name.at(i), pose);
//   }
//   sleep(1);
//   robot_state->updateTree();
}

void robotMeasurementCallback(const calibration_msgs::RobotMeasurement::Ptr robot_measurement)
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

//   static int current=0;
//   data->addMeasurement(robot_measurement);
//   data->showView(current);
//   current++;
}

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
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

  // create robot
  bool publising = true;  // true by default. It will publish /tf
  if (publising)
    robot_state = new RobotStatePublisher();
  else
    robot_state = new RobotState();

  // init robot from urdf
  robot_state->initFromURDF(model);

  // create node
  ros::NodeHandle n; //("calib");

  // subscriber
  ros::Subscriber subs_robot_measurement = n.subscribe("robot_measurement", 1,
                                                       robotMeasurementCallback);

  // visualization marker publisher
  visual_markers = new Markers();


  // read bag filename from param
  string rosbag_filename;
  if (!n.getParam("cal_measurements", rosbag_filename))
  {
    ROS_ERROR("Could read parameter cal_measurements on parameter server");
    return false;
  }

  bool offline = true; // offline by default (using bag file)
                       // 'online' method is not yet implemented
  if (offline)
  {
    // set Data class
    data = new Data();
    data->setRobotState(robot_state);
    data->setMarkers(visual_markers);

    // read rosbag
    rosbag::Bag bag(rosbag_filename);
    rosbag::View view(bag, rosbag::TopicQuery("robot_measurement"));
    vector<calibration_msgs::RobotMeasurement::Ptr> msgs;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      calibration_msgs::RobotMeasurement::Ptr i = m.instantiate<calibration_msgs::RobotMeasurement>();
      if (i != NULL)
      {
        data->addMeasurement(i);
      }
    }
    bag.close();


    // show views
//     size_t size = data->size();
//     for (size_t i=0; i<size; i++)
//     {
//       data->showView(i);
//       ros::spinOnce();
//       ros::Duration(0.5).sleep(); // sleep
//     }
//     data->showView(3);
//     ros::spinOnce();

    // Choose cameras to be calibrated
    std::vector<std::string> camera_frames;
    camera_frames.clear();
    camera_frames.push_back("narrow_stereo_l_stereo_camera_optical_frame"); // [I|0]
    camera_frames.push_back("narrow_stereo_r_stereo_camera_optical_frame");
    camera_frames.push_back("wide_stereo_l_stereo_camera_optical_frame");
    camera_frames.push_back("wide_stereo_r_stereo_camera_optical_frame");
    camera_frames.push_back("head_mount_kinect_rgb_optical_frame");
    camera_frames.push_back("high_def_optical_frame");

    // Optimization
    Optimization optimazer;
    optimazer.setRobotState(robot_state);
    optimazer.setMarkers(visual_markers);
    optimazer.setData(data);
    optimazer.setCamerasCalib(camera_frames);
    optimazer.run();
  }

  ros::spin();

  return 0;
}
