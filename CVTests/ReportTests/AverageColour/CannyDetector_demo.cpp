#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video/background_segm.hpp"
#include "sobol.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

/// Global variables

Mat tmp_frame, src_gray;
Mat dst, detected_edges;
Mat circle_mask;

VideoCapture cap;

int edgeThresh = 1;
int lowThreshold = 25;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;

Mat image, mask;
int loDiff = 0, upDiff = 0;
int connectivity = 4;
int newMaskVal = 255;
int flags = connectivity + (newMaskVal << 8) +
                FLOODFILL_FIXED_RANGE;

int dilation_type = MORPH_RECT;
int dilation_size = 1;

int blur_kernel = 3;
int max_blur_kernel = 10;
/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */

void CannyThreshold(int, void*)
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(blur_kernel,blur_kernel) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( detected_edges, dst, element );
  cvtColor(dst, dst, CV_GRAY2RGB); 
  // for(int i=0;i<dst.cols;i++)
  // {
  //   for(int j=0;j<dst.rows;j++)
  //   {
  //     if(dst.at<float>(i,j) == 0)
  //     {
  //       Point seed = Point(i,j);
  //       circle_mask = 255;
  //       circle(circle_mask,seed,50,(0,0,0),-1);
  //       Rect ccomp;
  //       int b = (unsigned)theRNG() & 255;
  //       int g = (unsigned)theRNG() & 255;
  //       int r = (unsigned)theRNG() & 255;
  //       Scalar newVal = Scalar(b,g,r);
  //       floodFill(dst, seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
  //                 Scalar(upDiff, upDiff, upDiff), flags);
  //     }
  //   }
  // }
  // findContours( dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

  for (unsigned long long i = 0; i < 400; ++i)
    {
        int x = tmp_frame.cols * sobol::sample(i, 0);
        int y = tmp_frame.rows * sobol::sample(i, 1);
        //cout << "sobol(" << i << ") = (" << x << ", " << y << ")" << endl;
        Point seed = Point(x, y);
        if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
        {
          circle_mask = 0;
          circle(circle_mask,seed,25,(255,255,255),-1);
          Scalar newVal = tmp_frame.at<Vec3b>(seed);
          Rect ccomp;
          floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                  Scalar(upDiff, upDiff, upDiff), flags);
        }
    }

  //tmp_frame.copyTo( dst, detected_edges);
  imshow( "Edge Map", dst );
 }


/** @function main */
int main( int argc, char** argv )
{
  cap.open(1);
  if( !cap.isOpened() )
  {
      printf("\nCan not open camera 1\n");
      cap.open(0);
      if( !cap.isOpened() )
      {
        printf("Can not open camera 0\n");
        return -1;
      }
  }
  cap.set(CV_CAP_PROP_FPS, 10);
  cap >> tmp_frame;
  if(tmp_frame.empty())
  {
      printf("can not read data from the video source\n");
      return -1;
  }

  namedWindow( "Edge Map", 1 );
  namedWindow("Camera", 1);
  cvMoveWindow( "Camera", tmp_frame.cols, 0 );
  createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
  createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, 0 );

  Point sample_point = Point(617,216);
  int frame_number = 120;
  int red[120], green[120], blue[120];

  for(int i = 0; i < frame_number; i++)
  {
    cap >> tmp_frame;
    if( tmp_frame.empty() )
        break;
    /// Create a matrix of the same type and size as src (for dst)
    dst.create( tmp_frame.size(), tmp_frame.type() );

    tmp_frame.copyTo(image);

    /// Convert the image to grayscale
    cvtColor( tmp_frame, src_gray, CV_BGR2GRAY );

    mask.create(tmp_frame.rows+2, tmp_frame.cols+2, CV_8UC1);
    circle_mask.create(tmp_frame.rows+2, tmp_frame.cols+2, CV_8UC1);
    
    /// Show the image
    CannyThreshold(0, 0);
    imshow("Camera", tmp_frame);
    red[i] = dst.at<Vec3b>(sample_point)[0];
    green[i] = dst.at<Vec3b>(sample_point)[1];
    blue[i] = dst.at<Vec3b>(sample_point)[2];
    char keycode = (char)waitKey(30);
      if( keycode == 27 )
          break;
  }
  for(int i=0;i<frame_number;i++)
  {
  	cout << red[i] << ", ";
  }
  cout << endl;
  for(int i=0;i<frame_number;i++)
  {
  	cout << green[i] << ", ";
  }
  cout << endl;
  for(int i=0;i<frame_number;i++)
  {
  	cout << blue[i] << ", ";
  }
  return 0;
  }