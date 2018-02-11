#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video/background_segm.hpp"
#include "sobol.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

using namespace cv;
using namespace std;

Mat tmp_frame, edge_detection;
Mat dst, flood_mask[4], mean_mask[4];
//Mat sub_dst[4];

thread t[3];
int finished = 0;
int imageReady[3] = {0,0,0};
mutex m;
condition_variable conVar;

struct SeedData
{
  Point seed;
  Scalar colour;
};

SeedData seedList0[25];
SeedData seedList1[25];
SeedData seedList2[25];
SeedData seedList3[25];

VideoCapture cap;
//Canny variables
int lowThreshold = 20;
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
//blur variables
int blur_kernel = 5;
int max_blur_kernel = 10;
//Final image saturation
int alpha = 1.2;

SeedData MakeSeedData(Point seed, Scalar colour)
{
  SeedData s;
  s.seed = seed;
  s.colour = colour;
  return s;
}

void FindColours(int corner, int offset_cols, int offset_rows, int sobolPoints)
{
  //Floodfill from quasi-random points
  //sub_dst[corner] = dst.clone();
  int k = 0;
  for (unsigned long long i = 0; i < sobolPoints; ++i)
  {
    int x = offset_cols + dst.cols/2 * sobol::sample(i, 0);
    int y = offset_rows + dst.rows * sobol::sample(i, 1);
    Point seed = Point(x, y);
    if(dst.at<Vec3b>(seed)[0] == 0)
    {
      flood_mask[corner] = 0;
      floodFill(dst, flood_mask[corner], seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
              Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8));
      for(int i=0;i<mean_mask[corner].rows;i++)
      {
        for(int j=0;j<mean_mask[corner].cols;j++)
        {
          mean_mask[corner].at<int>(i,j) = flood_mask[corner].at<int>(i+1,j+1);
        }
      }
      Scalar newVal = alpha*mean(tmp_frame,mean_mask[corner]);
      if(corner == 0) {
        //cout << seed << endl;
        seedList0[k] = MakeSeedData(seed, newVal);
        k++;
      }
      else if(corner == 1) {
        seedList1[k] = MakeSeedData(seed, newVal);
        k++;
        seedList1[k] = MakeSeedData(Point(0,0), Scalar(0,0,0));
      }
      else if(corner == 2) {
        seedList2[k] = MakeSeedData(seed, newVal);
        k++;
        seedList2[k] = MakeSeedData(Point(0,0), Scalar(0,0,0));
      }
      else if(corner == 3) {
        seedList3[k] = MakeSeedData(seed, newVal);
        k++;
      }
    }
  }
  if(corner == 0) seedList0[k] = MakeSeedData(Point(0,0), Scalar(0,0,0));
  else if(corner == 3) seedList3[k] = MakeSeedData(Point(0,0), Scalar(0,0,0));
}

void FindColoursThread(int corner, int sobolPoints)
{
  int offset_cols;
  int offset_rows;
  switch(corner) {
    case 0:
      offset_cols = 0;
      offset_rows = 0;
      break;
    case 1:
      offset_cols = tmp_frame.cols/2;
      offset_rows = 0;
      break;
    case 2:
      offset_cols = 0;
      offset_rows = tmp_frame.rows/2;
      break;
    case 3:
      offset_cols = tmp_frame.cols/2;
      offset_rows = tmp_frame.rows/2;
      break;
    default:
      cout << "FindColours: Unknown corner" << endl;
  }
  while(!finished)
  {
    unique_lock<mutex> lk(m);
    conVar.wait(lk, []{return imageReady[0];});
    //cout << "running" << endl;
    FindColours(corner, offset_cols, offset_rows, sobolPoints);
    imageReady[0] = 0;
    lk.unlock();
    conVar.notify_one();
  }
}
  

void CannyThreshold(int, void*)
{
  /// Reduce noise with kernel defined by blur_kernel
  blur( dst, dst, Size(blur_kernel,blur_kernel) );

  /// Canny detector
  Canny( dst, dst, lowThreshold, lowThreshold*thresh_ratio, kernel_size );

  /// Apply the dilation operation
  dilate( dst, dst, element );
  //erode( dst, dst, element );
  cvtColor(dst, dst, CV_GRAY2RGB);
  edge_detection = dst.clone(); 
  {
    lock_guard<mutex> lk(m);
    for(int i=0;i<3;i++) imageReady[i] = 1;
  }
  conVar.notify_one();
  FindColours(3, dst.cols/2, 0, 20);
  { 
    unique_lock<mutex> lk(m);
    conVar.wait(lk, []{return !imageReady[0];});
  }
  /*int j = 0;
  do
  {
    //cout << "filling" << endl;
    if(edge_detection.at<Vec3b>(seedList0[j].seed)[0] == 0 &&
      edge_detection.at<Vec3b>(seedList0[j].seed)[1] == 0 &&
      edge_detection.at<Vec3b>(seedList0[j].seed)[2] == 0)
    {
      floodFill(edge_detection, seedList0[j].seed, seedList0[j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
          Scalar(upDiff, upDiff, upDiff), flags);
    }
    j++;
  }while(seedList0[j].seed != Point(0,0));
  
  for(int j=0;j<seedList1.size();j++)
  {
    if(dst.at<Vec3b>(seedList1[j].seed)[0] == 0 && 
      dst.at<Vec3b>(seedList1[j].seed)[1] == 0 && 
      dst.at<Vec3b>(seedList1[j].seed)[2] == 0)
    { 
      floodFill(dst, seedList1[j].seed, seedList1[j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
          Scalar(upDiff, upDiff, upDiff), flags);
    }
  }
  for(int j=0;j<seedList2.size();j++)
  {
    if(dst.at<Vec3b>(seedList2[j].seed)[0] == 0 && 
      dst.at<Vec3b>(seedList2[j].seed)[1] == 0 && 
      dst.at<Vec3b>(seedList2[j].seed)[2] == 0)
    { 
      floodFill(dst, seedList2[j].seed, seedList2[j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
          Scalar(upDiff, upDiff, upDiff), flags);
    }
  }
  j = 0;
  while(seedList3[j].seed != Point(0,0))
  {
    //cout << "filling3" << endl;
    if(edge_detection.at<Vec3b>(seedList3[j].seed)[0] == 0 && 
      edge_detection.at<Vec3b>(seedList3[j].seed)[1] == 0 && 
      edge_detection.at<Vec3b>(seedList3[j].seed)[2] == 0)
    { 
      floodFill(edge_detection, seedList3[j].seed, seedList3[j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
          Scalar(upDiff, upDiff, upDiff), flags);
    }
    j++;
  }*/
  //cout << seedList0[0].colour << endl;
  //cout << seedList0[1].colour << endl;
  imshow( "Edge Map", dst );
 }

int main( int argc, char** argv )
{
  cap.open(0);
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
  cap.set(CV_CAP_PROP_FPS, 10);
  int FPS = cap.get(CV_CAP_PROP_FPS);
  cout << FPS << endl;
  cap >> tmp_frame;
  if(tmp_frame.empty())
  {
      printf("can not read data from the video source\n");
      return -1;
  }

  namedWindow( "Edge Map", 1 );
  cvMoveWindow( "Edge Map", 0, 40 );
  namedWindow("Camera", 1);
  cvMoveWindow( "Camera", tmp_frame.cols + 20, 40 );
  createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
  createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, 0 );

  for(int i=0;i<1;i++) t[i] = thread(FindColoursThread, i, 20);

  for(;;)
  {
    cap >> tmp_frame;
    if( tmp_frame.empty() )
        break;
    /// Create a matrix of the same type and size as src (for dst)
    for(int i=0;i<4;i++) flood_mask[i].create(tmp_frame.rows+2, tmp_frame.cols+2, CV_8UC1);
    for(int i=0;i<4;i++) mean_mask[i].create(tmp_frame.size(), CV_8UC1);

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
  finished = 1;
  for(int i=0;i<3;i++) t[i].join();
  return 0;
  }
