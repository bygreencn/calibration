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

namespace calib
{

ChessBoard::ChessBoard() : width_(0), height_(0), square_size_(0)
{
}

ChessBoard::~ChessBoard()
{
}

void ChessBoard::generateCorners(std::vector<cv::Point3d> *corners)
{
  // clear corners
  corners->clear();

  // generate corners: (x,y,z)
  for ( int i = 0; i < width_; i++ )
    for ( int j = 0; j < height_; j++ )
      corners->push_back( cv::Point3d( float(i*square_size_),
                                       float(j*square_size_), 0 ) );
}

void ChessBoard::generateCorners(cv::Mat_<double> *corners)
{
  std::vector<cv::Point3d> pts;
  generateCorners(&pts);
  *corners = cv::Mat(pts);
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

}
