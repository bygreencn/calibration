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

#include "triangulation.h"

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

namespace calib
{

template<typename T>
void homogeneousToEuclidean(const Mat & _X, Mat & _x)
{
    int d = _X.rows - 1;

    const Mat_<T> & X_rows = _X.rowRange(0,d);
    const Mat_<T> h = _X.row(d);

    const T * h_ptr = h[0], *h_ptr_end = h_ptr + h.cols;
    const T * X_ptr = X_rows[0];
    T * x_ptr = _x.ptr<T>(0);
    for (; h_ptr != h_ptr_end; ++h_ptr, ++X_ptr, ++x_ptr)
    {
        const T * X_col_ptr = X_ptr;
        T * x_col_ptr = x_ptr, *x_col_ptr_end = x_col_ptr + d * _x.step1();
        for (; x_col_ptr != x_col_ptr_end; X_col_ptr+=X_rows.step1(), x_col_ptr+=_x.step1() )
            *x_col_ptr = (*X_col_ptr) / (*h_ptr);
    }
}

void homogeneousToEuclidean(const InputArray _X, OutputArray _x)
{
    // src
    const Mat X = _X.getMat();

    // dst
     _x.create(X.rows-1, X.cols, X.type());
    Mat x = _x.getMat();

    // type
    if( X.depth() == CV_32F )
    {
        homogeneousToEuclidean<float>(X,x);
    }
    else
    {
        homogeneousToEuclidean<double>(X,x);
    }
}

// It is the standard DLT (for multiple views)
void nViewTriangulate(const Mat_<double> &x, const vector<Matx34d> &Ps, Vec3d &X)
{
  CV_Assert(x.rows == 2);
  unsigned nviews = x.cols;
  CV_Assert(nviews == Ps.size());

  cv::Mat_<double> design = cv::Mat_<double>::zeros(2 * nviews, 4);
  for (int i = 0; i < nviews; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      design(i*2,   j) = x(0,i) * Ps[i](2, j) - Ps[i](0, j);
      design(i*2+1, j) = x(1,i) * Ps[i](2, j) - Ps[i](1, j);
    }
  }

  Mat X_homog;
  cv::SVD::solveZ(design, X_homog);
  homogeneousToEuclidean(X_homog, X);
}

}
