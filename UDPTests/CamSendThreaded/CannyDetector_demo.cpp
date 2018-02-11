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
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace cv;
using namespace std;

#define SERVICE_PORT  21234
#define BUFLEN 40960

Mat tmp_frame, dst_out;
Mat dst, flood_mask[4], mean_mask[4];
//Mat sub_dst[4];

thread t;
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

struct sockaddr_in myaddr, remaddr;
int fd, bufSize, k, slen=sizeof(remaddr);
char server[] = "10.9.177.131"; /* change this to use a different server */
uchar buf[BUFLEN];
uchar buf1[1024];
int bufLength;
int buf1Length;
vector<uchar> dstBuf;

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
  int j = 0;
  for (unsigned long long i = 0; i < sobolPoints; ++i)
  {
    int x = offset_cols + dst.cols/2 * sobol::sample(i, 0);
    int y = offset_rows + dst.rows * sobol::sample(i, 1);
    //cout << corner << "1\n";
    Point seed = Point(x, y);
    if(dst.at<Vec3b>(seed)[0] == 0)
    {
      flood_mask[corner] = 0;
      floodFill(dst, flood_mask[corner], seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
              Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8));
      for(int i=0;i<mean_mask[corner].rows;i++)
      {
        for(int a=0;a<mean_mask[corner].cols;a++)
        {
          mean_mask[corner].at<int>(i,a) = flood_mask[corner].at<int>(i+1,a+1);
        }
      }
      Scalar newVal = alpha*mean(tmp_frame,mean_mask[corner]);
      //cout << corner << endl;
      if(corner == 0) {
        buf1[j*7] = (x >> 8) & 0xFF;
        buf1[j*7+1] = x & 0xFF;
        buf1[j*7+2] = (y >> 8) & 0xFF;
        buf1[j*7+3] = y & 0xFF;
        buf1[j*7+4] = newVal[0];
        buf1[j*7+5] = newVal[1];
        buf1[j*7+6] = newVal[2];
        j++;
      }
      else if(corner == 3) {
        buf[j*7] = (x >> 8) & 0xFF;
        buf[j*7+1] = x & 0xFF;
        buf[j*7+2] = (y >> 8) & 0xFF;
        buf[j*7+3] = y & 0xFF;
        buf[j*7+4] = newVal[0];
        buf[j*7+5] = newVal[1];
        buf[j*7+6] = newVal[2];
        j++;
      }
    }
  }
  if(corner == 0) {
    buf1Length = j;
  }
  else if(corner == 3) {
    bufLength = j;
  }
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
  dst_out = dst.clone(); 
  {
    lock_guard<mutex> lk(m);
    for(int i=0;i<3;i++) imageReady[i] = 1;
  }
  conVar.notify_one();
  FindColours(3, dst.cols/2, 0, 20);
  //cout << "0\n";
  { 
    unique_lock<mutex> lk(m);
    conVar.wait(lk, []{return !imageReady[0];});
  }
  //cout << "1\n";
  for(int i=0;i<buf1Length*7;i++) buf[bufLength*7 + i] = buf1[i];
  for(int i=0;i<7;i++) buf[bufLength*7 + buf1Length*7 + i] = 0;
  resize(dst_out, dst_out, Size(), 0.5, 0.5, CV_INTER_AREA);
  imencode(".png", dst_out, dstBuf);
  cout << "2\n";
  for(int i=0;i<dstBuf.size();i++) buf[bufLength*7 + buf1Length*7 + i] = dstBuf[i];

  imshow( "Edge Map", dst );
 }

int main( int argc, char** argv )
{
  /* create a socket */

  if ((fd=socket(AF_INET, SOCK_DGRAM, 0))==-1)
    printf("socket created\n");

  /* bind it to all local addresses and pick any port number */

  memset((char *)&myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  myaddr.sin_port = htons(0);

  if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
    perror("bind failed");
    return 0;
  }       

  /* now define remaddr, the address to whom we want to send messages */
  /* For convenience, the host address is expressed as a numeric IP address */
  /* that we will convert to a binary format via inet_aton */

  memset((char *) &remaddr, 0, sizeof(remaddr));
  remaddr.sin_family = AF_INET;
  remaddr.sin_port = htons(SERVICE_PORT);
  if (inet_aton(server, &remaddr.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }

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
  //createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
  //createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, 0 );

  //for(int i=0;i<1;i++) t[i] = thread(FindColoursThread, i, 20);
  t = thread(FindColoursThread, 0, 20);

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

    if (sendto(fd, buf, bufSize, 0, (struct sockaddr *)&remaddr, slen)==-1)
      perror("sendto");

    char keycode = (char)waitKey(30);
      if( keycode == 27 ){
          //imwrite("frame.jpg", dst);
          break;
        }
  }
  finished = 1;
  //for(int i=0;i<3;i++) t[i].join();
  t.join();
  return 0;
  }
