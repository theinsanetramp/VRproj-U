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

Mat tmp_frame, edge_detection, dst;
Mat flood_mask, mean_mask;

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
Rect ccomp;
int flags = connectivity + (newMaskVal << 8) +
                FLOODFILL_FIXED_RANGE;
//dilation variables
int dilation_type = MORPH_RECT;
int dilation_size = 1;
Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
//blur variables
int blur_kernel = 5;
int max_blur_kernel = 10;

int x, y, i, j;
Point seed;
Scalar avg_colour;

void CannyThreshold(int, void*)
{
  /// Reduce noise with kernel defined by blur_kernel
  blur( dst, dst, Size(blur_kernel,blur_kernel) );

  /// Canny detector
  Canny( dst, dst, lowThreshold, lowThreshold*ratio, kernel_size );
  /// Apply the dilation operation
  dilate( dst, dst, element ); 
  //cvtColor(dst, dst, CV_GRAY2RGB); 
  edge_detection = dst.clone();
  
  #ifdef SOBOL
  //Floodfill from quasi-random points
  for (unsigned long long i = 0; i < 50; ++i)
  {
      x = tmp_frame.cols * sobol::sample(i, 0);
      y = tmp_frame.rows * sobol::sample(i, 1);
      if(dst.at<int>(y,x) == 0)
      {
	flood_mask = 0;
        floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8));
	for(i=0;i<mean_mask.rows;i++)
	{
		for(j=0;j<mean_mask.cols;j++)
		{
			mean_mask.at<int>(i,j) = flood_mask.at<int>(i+1,j+1);
		}
	}
        avg_colour = mean(tmp_frame,mean_mask);
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
        Rect ccomp;
        floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
        Scalar newVal = mean(tmp_frame,resize(flood_mask,flood_mask);
      }
    }
  }
  #endif
  
  imshow( "Edge Map", edge_detection );
 }

int main( int argc, char** argv )
{
  cap.open(0);
  if( !cap.isOpened() )
  {
      cout << "\nCan not open camera 1\n";
      cap.open(0);
      if( !cap.isOpened() )
      {
        cout << "Can not open camera 0\n";
        return -1;
      }
  }
  //cap.set(CV_CAP_PROP_FRAME_WIDTH,494);
  //cap.set(CV_CAP_PROP_FRAME_HEIGHT,768);
  //cap.set(CV_CAP_PROP_FPS, 10);
  //int FPS = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  //cout << FPS << endl;
  cap >> tmp_frame;
  if(tmp_frame.empty())
  {
      cout << "can not read data from the video source\n";
      return -1;
  }

  namedWindow( "Edge Map", 1 );
  namedWindow("Camera", 1);
  cvMoveWindow( "Camera", tmp_frame.cols, 0 );
  //createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
  //createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, 0 );

  for(;;)
  {
    cap >> tmp_frame;
    if( tmp_frame.empty() )
        break;
    /// Create a matrix of the same type and size as src (for dst)
    dst.create( tmp_frame.size(), tmp_frame.type() );
    edge_detection.create(tmp_frame.size(), CV_8UC1);
    flood_mask.create(tmp_frame.rows+2, tmp_frame.cols+2, CV_8UC1);
    mean_mask.create(tmp_frame.rows, tmp_frame.cols, CV_8UC1);

    /// Convert the image to grayscale
    cvtColor( tmp_frame, dst, CV_BGR2GRAY );

    /// Show the image
    CannyThreshold(0, 0);
    imshow("Camera", tmp_frame);
    char keycode = (char)waitKey(30);
      if( keycode == 27 ){
          //imwrite("frame.jpg", dst);
          break;
        }
  }
  return 0;
  }
