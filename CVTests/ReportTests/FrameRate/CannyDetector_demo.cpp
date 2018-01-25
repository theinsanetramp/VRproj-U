#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video/background_segm.hpp"
#include "sobol.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <time.h>

using namespace cv;
using namespace std;

//#define SOBOL

Mat tmp_frame, src_gray;
Mat dst, detected_edges;
Mat flood_mask;

VideoCapture cap;
//Canny variables
int lowThreshold = 24;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
//floodFill variables
int loDiff = 0, upDiff = 0;
int connectivity = 4;
int newMaskVal = 255;
int flags = connectivity + (newMaskVal << 8) +
                FLOODFILL_FIXED_RANGE;
//dilation variables
int dilation_type = MORPH_RECT;
int dilation_size = 1;
//blur variables
int blur_kernel = 3;
int max_blur_kernel = 10;
//Final image saturation
int alpha = 1.2;

void CannyThreshold(int, void*)
{
  /// Reduce noise with kernel defined by blur_kernel
  blur( src_gray, detected_edges, Size(blur_kernel,blur_kernel) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( detected_edges, dst, element );
  cvtColor(dst, dst, CV_GRAY2RGB); 

  #ifdef SOBOL
  //Floodfill from quasi-random points
  for (unsigned long long i = 0; i < 400; ++i)
  {
      int x = tmp_frame.cols * sobol::sample(i, 0);
      int y = tmp_frame.rows * sobol::sample(i, 1);
      Point seed = Point(x, y);
      if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
      {
        flood_mask = 0;
        Rect ccomp;
        floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
        Scalar newVal = mean(tmp_frame,flood_mask);
        floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), flags);
      }
  }
  for( int y = 0; y < dst.rows; y++ ) {
      for( int x = 0; x < dst.cols; x++ ) {
          for( int c = 0; c < 3; c++ ) {
              dst.at<Vec3b>(y,x)[c] =
                saturate_cast<uchar>( alpha*( dst.at<Vec3b>(y,x)[c] ));
          }
      }
  }
  #else
  //Floodfill every empty area
  for(int i=0;i<dst.cols;i++)
  {
    for(int j=0;j<dst.rows;j++)
    {
      Point seed = Point(i,j);
      if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
      {
        flood_mask = 0;
        Rect ccomp;
        floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
        Scalar newVal = mean(tmp_frame,flood_mask);
        floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), flags);
      }
      for( int c = 0; c < 3; c++ ) {
          dst.at<Vec3b>(j,i)[c] =
            saturate_cast<uchar>( alpha*( dst.at<Vec3b>(j,i)[c] ));
      }
    }
  }
  #endif

  imshow( "Edge Map", dst );
 }

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

  int num_frames = 120;
  time_t start, end;

  namedWindow( "Edge Map", 1 );
  namedWindow("Camera", 1);
  cvMoveWindow( "Camera", tmp_frame.cols, 0 );
  createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
  createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, 0 );

  time(&start);

  for(int i = 0; i < num_frames; i++)
  {
    cap >> tmp_frame;
    if( tmp_frame.empty() )
        break;
    /// Create a matrix of the same type and size as src (for dst)
    dst.create( tmp_frame.size(), tmp_frame.type() );
    flood_mask.create(tmp_frame.rows+2, tmp_frame.cols+2, CV_8UC1);

    /// Convert the image to grayscale
    cvtColor( tmp_frame, src_gray, CV_BGR2GRAY );

    /// Show the image
    CannyThreshold(0, 0);
    imshow("Camera", tmp_frame);
    char keycode = (char)waitKey(30);
      if( keycode == 27 )
          break;
  }
  time(&end);
  double seconds = difftime(end, start);
  cout << "Time taken: " << seconds << " seconds" << endl;
  double fps = num_frames/seconds;
  cout << "FPS: " << fps << endl;
  return 0;
  }