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


#ifndef CHESS_BOARD_H
#define CHESS_BOARD_H

#include <opencv2/core/core.hpp>

namespace calib
{

class ChessBoard
{
public:
  ChessBoard();
  ~ChessBoard();

  /// \brief Check if it is empty (valid)
  bool empty();

  /// \brief Generates ChessBoard corners, 3D points assuming Z=0
  /// \For simplicity, the coordinate system has been chosen such 
  /// \that one of the chessboard corners is in the origin and the
  /// \board is in the plane Z=0.
  void generateCorners(std::vector<cv::Point3d> *corners);

  /// get functions
  int getWidth()      { return width_;}
  int getHeight()     { return height_;}
  int getSquareSize() { return square_size_;}

  /// set function
  void setSize(int width, int height, float square_size);

private:
  int   width_;        // corners_x
  int   height_;       // corners_y
  float square_size_;  // meters, e.i: 0.108 => 108 mm
};

}

#endif // CHESS_BOARD_H

