#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>
#include <stdint.h>
#include "elas.h"

using namespace cv;
using namespace std;

Mat generateDisparityMap(Mat& left, Mat& right) {
  if (left.empty() || right.empty())
    return left;
  const Size imsize = left.size();
  const int32_t dims[3] = {imsize.width,imsize.height,imsize.width};
  Mat leftdpf = Mat::zeros(imsize, CV_32F);
  Mat rightdpf = Mat::zeros(imsize, CV_32F);

  Elas::parameters param(Elas::ROBOTICS);
  param.postprocess_only_left = true;
  param.ipol_gap_width = 10;
  Elas elas(param);
  elas.process(left.data,right.data,leftdpf.ptr<float>(0),
               rightdpf.ptr<float>(0),dims);

  Mat show = Mat(imsize, CV_8UC1, Scalar(0));
  leftdpf.convertTo(show, CV_8U, 1.);
  return show;
}

int main(int argc, char** argv)
{
    int wsize = 15;
    int max_disp = 80;
    double lambda = 8000;
    double sigma = 1.5;
    double vis_mult = 1;

    //! [load_views]
    Mat left  = imread(argv[1], IMREAD_COLOR);
    if ( left.empty() )
    {
        cout<<"Cannot read image file: "<<argv[1];
        return -1;
    }

    Mat right = imread(argv[2], IMREAD_COLOR);
    if ( right.empty() )
    {
        cout<<"Cannot read image file: "<<argv[2];
        return -1;
    }
    cvtColor(left,  left,  COLOR_BGR2GRAY);
    cvtColor(right, right, COLOR_BGR2GRAY);
    double matching_time;
    matching_time = (double)getTickCount();
    Mat final = generateDisparityMap(left, right);
    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

    //collect and print all the stats:
    cout.precision(2);
    cout<<"Matching time:  "<<matching_time<<"s"<<endl;

    imwrite("result.jpg",final);
    
    namedWindow("left", WINDOW_AUTOSIZE);
    imshow("left", left);
    namedWindow("right", WINDOW_AUTOSIZE);
    imshow("right", right);


    namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    imshow("filtered disparity", final);
    waitKey();

    return 0;
}