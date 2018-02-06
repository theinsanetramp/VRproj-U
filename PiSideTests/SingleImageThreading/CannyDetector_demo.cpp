#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "sobol.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <vector>

using namespace cv;
using namespace std;

Mat src;
Mat dst, flood_mask[4], mean_mask[4];
Mat sub_dst[4];

thread t[3];

struct SeedData
{
  Point seed;
  Scalar colour;
};

vector<vector<SeedData>> seedLists;

//Canny variables
int lowThreshold = 25;
int const max_lowThreshold = 100;
int thresh_ratio = 3;
int kernel_size = 3;
//floodFill variables
int loDiff = 0, upDiff = 0;
int connectivity = 4;
int newMaskVal = 255;
int flags = connectivity + (newMaskVal << 8) +
                FLOODFILL_FIXED_RANGE;
Rect ccomp;
//dilation variables
int dilation_type = MORPH_RECT;
int dilation_size = 1;
Mat element = getStructuringElement( dilation_type,
	                           Size( 2*dilation_size + 1, 2*dilation_size+1 ),
	                           Point( dilation_size, dilation_size ) );

SeedData MakeSeedData(Point seed, Scalar colour)
{
  SeedData s;
  s.seed = seed;
  s.colour = colour;
  return s;
}

void FindColours(int corner, int sobolPoints)
{
  dst.copyTo(sub_dst[corner]);
  int offset_cols;
  int offset_rows;
  switch(corner) {
    case 0:
      offset_cols = 0;
      offset_rows = 0;
      break;
    case 1:
      offset_cols = sub_dst[corner].cols/2;
      offset_rows = 0;
      break;
    case 2:
      offset_cols = 0;
      offset_rows = sub_dst[corner].rows/2;
      break;
    case 3:
      offset_cols = sub_dst[corner].cols/2;
      offset_rows = sub_dst[corner].rows/2;
      break;
    default:
      cout << "FindColours: Unknown corner" << endl;
  }

  //Floodfill from quasi-random points
  for (unsigned long long i = 0; i < sobolPoints; ++i)
  {
    int x = offset_cols + sub_dst[corner].cols/2 * sobol::sample(i, 0);
    int y = offset_rows + sub_dst[corner].rows/2 * sobol::sample(i, 1);
    Point seed = Point(x, y);
    if(sub_dst[corner].at<Vec3b>(seed)[0] == 0 && 
      sub_dst[corner].at<Vec3b>(seed)[1] == 0 && 
      sub_dst[corner].at<Vec3b>(seed)[2] == 0)
    {
      flood_mask[corner] = 0;
      floodFill(sub_dst[corner], flood_mask[corner], seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
              Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8));
      for(int i=0;i<mean_mask[corner].rows;i++) {
        for(int j=0;j<mean_mask[corner].cols;j++) {
          mean_mask[corner].at<int>(i,j) = flood_mask[corner].at<int>(i+1,j+1);
        }
      }
      Scalar newVal = mean(src,mean_mask[corner]);
      seedLists[corner].push_back(MakeSeedData(seed, newVal));
    }
  }
}

void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 5x5
  blur( dst, dst, Size(5,5) );
  //imwrite("buildBlur.jpg",detected_edges);
  /// Canny detector
  Canny( dst, dst, lowThreshold, lowThreshold*thresh_ratio, kernel_size );
  //imwrite("buildCanny.jpg",detected_edges);
  /// Apply the dilation operation
  dilate( dst, dst, element );
  //imwrite("buildDilated.jpg",dst);
  cvtColor(dst, dst, CV_GRAY2RGB); 
  seedLists.resize(4);
  for(int i=0;i<3;i++) t[i] = thread(FindColours, i, 600);
  FindColours(3, 600);
  for(int i=0;i<3;i++) t[i].join();
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<seedLists[i].size();j++)
    {
      if(dst.at<Vec3b>(seedLists[i][j].seed)[0] == 0 && 
        dst.at<Vec3b>(seedLists[i][j].seed)[1] == 0 && 
        dst.at<Vec3b>(seedLists[i][j].seed)[2] == 0)
      { 
        floodFill(dst, seedLists[i][j].seed, seedLists[i][j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
            Scalar(upDiff, upDiff, upDiff), flags);
      }
    }
  }

  imshow( "Edge Map", sub_dst[0] );
  imshow( "Edge Map1", sub_dst[1] );
  imshow( "Edge Map2", sub_dst[2] );
  imshow( "Edge Map3", sub_dst[3] );
  imshow( "Edge Map4", dst );
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
  for(int i=0;i<4;i++) flood_mask[i].create(src.rows+2, src.cols+2, CV_8UC1);

  /// Convert the image to grayscale
  cvtColor( src, dst, CV_BGR2GRAY );
  //imwrite("buildGray.jpg",src_gray);
  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map", 0, 40 );
  namedWindow( "Edge Map1", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map1", src.cols + 40, 40 );
  namedWindow( "Edge Map2", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map2", 0, src.rows + 80 );
  namedWindow( "Edge Map3", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map3", src.cols + 40, src.rows + 80 );
  namedWindow( "Edge Map4", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map4", 2*src.cols + 40, 40 );

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  int k;
  do{
  	 /// Wait until user exit program by pressing a key
  	 k = waitKey(0);
  }
  while(k != 27);
  //imwrite("Final.bmp",dst);
  return 0;
  }
