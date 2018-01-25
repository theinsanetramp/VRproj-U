#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "sobol.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

#define SOBOL

Mat src, src_gray;
Mat dst, detected_edges;
Mat flood_mask;

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
//dilation variables
int dilation_type = MORPH_RECT;
int dilation_size = 1;

int blur_kernel = 5;
int max_blur_kernel = 10;

void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 5x5
  blur( src_gray, detected_edges, Size(blur_kernel,blur_kernel) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);
  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( detected_edges, dst, element );
  // cvtColor(dst, dst, CV_GRAY2RGB); 

  // #ifdef SOBOL
  // //Floodfill from quasi-random points
  // for (unsigned long long i = 0; i < 600; ++i)
  //   {
  //       int x = src.cols * sobol::sample(i, 0);
  //       int y = src.rows * sobol::sample(i, 1);
  //       Point seed = Point(x, y);
  //       if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
  //       {
  //         flood_mask = 0;
  //         Rect ccomp;
  //         floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
  //                 Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
  //         Scalar newVal = mean(src,flood_mask);
  //         floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
  //                 Scalar(upDiff, upDiff, upDiff), flags);
  //       }
  //   }
  // #else
  // //Floodfill every empty area
  // for(int i=0;i<dst.cols;i++)
  // {
  //   for(int j=0;j<dst.rows;j++)
  //   {
  //     Point seed = Point(i,j);
  //     if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
  //     {
  //       flood_mask = 0;
  //       Rect ccomp;
  //       floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
  //               Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
  //       Scalar newVal = mean(src,flood_mask);
  //       floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
  //               Scalar(upDiff, upDiff, upDiff), flags);
  //     }
  //   }
  // }
  // #endif
  //imwrite("orangeThresh45.jpg",dst);
  imshow( "Edge Map", dst );
 }


/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );
  flood_mask.create(src.rows+2, src.cols+2, CV_8UC1);

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  moveWindow("Edge Map", 100,100);

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
  createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  int k;
  do{
  	 /// Wait until user exit program by pressing a key
  	 k = waitKey(0);
  }
  while(k != 27);

  return 0;
  }