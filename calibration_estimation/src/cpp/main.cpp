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
 * This is a app for run and visualize estimation errors
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include "optimization.h"

#include "markers.h"
#include "robot_state.h"
#include "robot_state_publisher.h"
#include "calibration_msgs/RobotMeasurement.h"

using namespace std;
using namespace cv;
using namespace calib;

// global variables
RobotState *robot_state;
Markers    *visual_markers;
Data       *data;

Optimization optimazer;
map<string, int> sample_id;

void robotMeasurementCallback(const calibration_msgs::RobotMeasurement::Ptr robot_measurement)
{
  data->showView( sample_id[robot_measurement->sample_id] );
}

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "estimation");

  // read urdf model from ROS param
  urdf::Model model;
  if (!model.initParam("robot_description"))
    return EXIT_FAILURE;

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
    int id=0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      calibration_msgs::RobotMeasurement::Ptr i = m.instantiate<calibration_msgs::RobotMeasurement>();
      if (i != NULL)
      {
        sample_id[i->sample_id] = id;
        data->addMeasurement(i);
        id++;
      }
    }
    bag.close();

    // subscriber (for debug)
//   ros::Subscriber subs_robot_measurement = n.subscribe("robot_measurement", 1,
//                                                        robotMeasurementCallback);

    // show views (for debug)
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
    optimazer.setRobotState(robot_state);
    optimazer.setMarkers(visual_markers);
    optimazer.setData(data);
    optimazer.setCamerasCalib(camera_frames);
    optimazer.run();
  }

  // save urdf to file
//   TiXmlDocument* output = urdf::exportURDF( model );
//   if (!output)
//     ROS_ERROR("Failed to save urdf file\n");
//   else
//     output->SaveFile("urdf_calibrated.xml");

  ros::spin();

  return 0;
}
