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

#include "chessboard.h"
#include "projection.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>

using namespace std;
using namespace cv;

namespace calib
{

ChessBoard::ChessBoard() : width_(0), height_(0), square_size_(0)
{
}

ChessBoard::~ChessBoard()
{
}

void ChessBoard::generateCorners(vector<Point3d> *corners)
{
  // clear corners
  corners->clear();

  // generate corners: (x,y,z)
  for ( int j = 0; j < height_; j++ )
    for ( int i = 0; i < width_; i++ )
      corners->push_back( Point3d( float(i*square_size_),
                                       float(j*square_size_), 0 ) );
}

void ChessBoard::generateCorners(Mat_<double> *corners)
{
  vector<Point3d> pts;
  generateCorners(&pts);
  *corners = Mat(pts);
  transpose(*corners, *corners);
}

bool ChessBoard::empty(void)
{
  return width_ == 0 || height_ == 0 || square_size_ == 0;
}

void ChessBoard::setSize(int width, int height, float square_size)
{
  width_       = width;
  height_      = height;
  square_size_ = square_size;
}


double findChessboardPose(InputArray objectPoints,
                          InputArray imagePoints,
                          InputArray cameraMatrix,
                          InputArray distCoeffs,
                          OutputArray rvec,
                          OutputArray tvec,
                          OutputArray proj_points2D)
{
  solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
               rvec, tvec, false, CV_ITERATIVE);

  // reprojection error
  double err = computeReprojectionErrors(objectPoints, imagePoints,
                                         cameraMatrix, distCoeffs,
                                         rvec, tvec, proj_points2D);

  return err;
}

void getCheckboardSize(const string &target_id, ChessBoard *cb)
{
  if (target_id == "large_cb_7x6")
  {
    cb->setSize(7, 6, 0.108);
  }
  else if (target_id == "small_cb_4x5")
  {
    cb->setSize(4, 5, 0.0245);
  }
  else
    ROS_ERROR("Wrong Checkboard size");
}

}
