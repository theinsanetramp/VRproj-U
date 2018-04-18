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

#define SOBOL

Mat tmp_frame, detected_edges;
Mat dst, flood_mask;

VideoCapture cap;
//Canny variables
int lowThreshold = 20;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
//floodFill variables
int loDiff = 0, upDiff = 0;
int connectivity = 4;
int newMaskVal = 255;
int flags = connectivity + (newMaskVal << 8) +
                FLOODFILL_FIXED_RANGE;
Rect ccomp;
Point seed;
//dilation variables
int dilation_type = MORPH_RECT;
int dilation_size = 1;
Mat element = getStructuringElement( dilation_type,
                                   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                   Point( dilation_size, dilation_size ) );
//blur variables
int blur_kernel = 5;
int max_blur_kernel = 10;
//Final image saturation
int alpha = 1.2;

void CannyThreshold(int, void*)
{
  /// Canny detector
  Canny( dst, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Apply the dilation operation
  dilate( detected_edges, detected_edges, element );
  cvtColor(detected_edges, detected_edges, CV_GRAY2RGB); 

  #ifdef SOBOL
  //Floodfill from quasi-random points
  for (unsigned long long i = 0; i < 400; ++i)
  {
      int x = tmp_frame.cols * sobol::sample(i, 0);
      int y = tmp_frame.rows * sobol::sample(i, 1);
      seed = Point(x,y);
      if(detected_edges.at<Vec3b>(seed)[0] == 0 && detected_edges.at<Vec3b>(seed)[1] == 0 && detected_edges.at<Vec3b>(seed)[2] == 0)
      {
        flood_mask = 0;
        floodFill(detected_edges, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
        Scalar newVal = alpha*mean(tmp_frame,flood_mask);
        floodFill(detected_edges, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), flags);
      }
  }
  #else
  //Floodfill every empty area
  for(int i=0;i<detected_edges.cols;i++)
  {
    for(int j=0;j<detected_edges.rows;j++)
    {
      Point seed = Point(i,j);
      if(detected_edges.at<Vec3b>(seed)[0] == 0 && detected_edges.at<Vec3b>(seed)[1] == 0 && detected_edges.at<Vec3b>(seed)[2] == 0)
      {
        flood_mask = 0;
        floodFill(detected_edges, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
        Scalar newVal = alpha*mean(tmp_frame,flood_mask);
        floodFill(detected_edges, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), flags);
      }
    }
  }
  #endif

  imshow( "Edge Map", detected_edges );
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
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,494);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,768);
  //cap.set(CV_CAP_PROP_FPS, 40);
  int FPS = cap.get(CV_CAP_PROP_FPS);
  cout << FPS << endl;
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
  //createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, 0 );

  for(;;)
  {
    cap >> tmp_frame;
    if( tmp_frame.empty() )
        break;
    /// Create a matrix of the same type and size as src (for dst)
    dst.create( tmp_frame.size(), tmp_frame.type() );
    flood_mask.create(tmp_frame.rows+2, tmp_frame.cols+2, CV_8UC1);

    /// Convert the image to grayscale
    cvtColor( tmp_frame, dst, CV_BGR2GRAY );
    // Reduce noise with kernel defined by blur_kernel
    blur( dst, dst, Size(blur_kernel,blur_kernel) );

    /// Show the image
    CannyThreshold(0, 0);
    imshow("Camera", tmp_frame);
    char keycode = (char)waitKey(30);
      if( keycode == 27 ){
          //imwrite("frame.jpg", detected_edges);
          break;
        }
  }
  return 0;
  }