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

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;
Mat circle_mask, dilated_detected_edges;

int edgeThresh = 1;
int lowThreshold = 20;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;

Mat image, mask;
int loDiff = 0, upDiff = 0;
int connectivity = 4;
int isColor = true;
int newMaskVal = 255;
int flags = connectivity + (newMaskVal << 8) +
                FLOODFILL_FIXED_RANGE;

int dilation_type = MORPH_RECT;
int dilation_size = 1;

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */

static void onMouse( int event, int x, int y, int, void* )
{
    if( event != EVENT_LBUTTONDOWN )
        return;
    Point seed = Point(x,y);
    circle_mask = 0;
    circle(circle_mask,seed,25,(255,255,255),-1);
    //imshow("Mask",circle_mask);
    Scalar newVal = mean(image,circle_mask);
    //int b = (unsigned)theRNG() & 255;
    //int g = (unsigned)theRNG() & 255;
    //int r = (unsigned)theRNG() & 255;
    Rect ccomp;
    //Scalar newVal = isColor ? Scalar(b, g, r) : Scalar(r*0.299 + g*0.587 + b*0.114);
    int area = floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                  Scalar(upDiff, upDiff, upDiff), flags);
    imshow("Edge Map", dst);
    cout << area << " pixels were repainted\n";
}

void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(5,5) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);
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
        // circle_mask = 255;
        // circle(circle_mask,seed,50,(0,0,0),-1);
        // Rect ccomp;
        // int b = (unsigned)theRNG() & 255;
        // int g = (unsigned)theRNG() & 255;
        // int r = (unsigned)theRNG() & 255;
        // Scalar newVal = Scalar(b,g,r);
        // floodFill(dst, seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
        //           Scalar(upDiff, upDiff, upDiff), flags);
  //     }
  //   }
  // }
  //src.copyTo( dst, dilated_detected_edges);

  for (unsigned long long i = 0; i < 600; ++i)
    {
        int x = image.cols * sobol::sample(i, 0);
        int y = image.rows * sobol::sample(i, 1);
        //cout << "sobol(" << i << ") = (" << x << ", " << y << ")" << endl;
        Point seed = Point(x, y);
        if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
        {
          circle_mask = 0;
          circle(circle_mask,seed,25,(255,255,255),-1);
          Scalar newVal = mean(image,circle_mask);
          Rect ccomp;
          floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                  Scalar(upDiff, upDiff, upDiff), flags);
        }
    }

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

  circle_mask.create(src.rows+2, src.cols+2, CV_8UC1);

  src.copyTo(image);

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  mask.create(src.rows+2, src.cols+2, CV_8UC1);
  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  setMouseCallback( "Edge Map", onMouse, 0 );
  //namedWindow("Mask",1);

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  waitKey(0);

  return 0;
  }