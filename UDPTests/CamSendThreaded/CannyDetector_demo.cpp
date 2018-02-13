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

#define FRAMERATE 10
#define POINTSPERTHREAD 20

Mat tmp_frame, dst_out;
Mat dst, flood_mask[4], mean_mask[4];
//Mat sub_dst[4];

thread t[3];
int finished = 0;
int imageReady0 = 0;
int imageReady1 = 0;
int imageReady2 = 0;
mutex m;
condition_variable conVar;
condition_variable conVar1;
condition_variable conVar2;

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
int blur_kernel = 3;
int max_blur_kernel = 10;
//Final image saturation
int alpha = 1.2;

struct sockaddr_in myaddr, remaddr;
int fd, bufSize, k, slen=sizeof(remaddr);
//char server[] = "10.42.0.1";
uchar buf[BUFLEN];
uchar buf0[1024];
uchar buf1[1024];
uchar buf2[1024];
int bufLength;
int buf0Length;
int buf1Length;
int buf2Length;
vector<uchar> dstBuf;

void FindColours(int corner, int offset_cols, int offset_rows, int sobolPoints)
{
  //Floodfill from quasi-random points
  //sub_dst[corner] = dst.clone();
  int j = 0;
  for (unsigned long long i = sobolPoints*corner; i < sobolPoints*(corner+1); ++i)
  {
    int x = dst.cols * sobol::sample(i, 0);
    int y = dst.rows * sobol::sample(i, 1);
    Point seed = Point(x, y);
    //circle(dst, seed, 5, 255, -1);
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
      for(int i=0;i<3;i++) if(newVal[i] < 1) newVal[i] = 1;
      if(corner == 0) {
        //cout << seed << "  " << newVal <<  endl;
        buf0[j*7] = (x >> 8) & 0xFF;
        buf0[j*7+1] = x & 0xFF;
        buf0[j*7+2] = (y >> 8) & 0xFF;
        buf0[j*7+3] = y & 0xFF;
        buf0[j*7+4] = newVal[0];
        buf0[j*7+5] = newVal[1];
        buf0[j*7+6] = newVal[2];
        j++;
      }
      if(corner == 1) {
        //cout << seed << "  " << newVal <<  endl;
        buf1[j*7] = (x >> 8) & 0xFF;
        buf1[j*7+1] = x & 0xFF;
        buf1[j*7+2] = (y >> 8) & 0xFF;
        buf1[j*7+3] = y & 0xFF;
        buf1[j*7+4] = newVal[0];
        buf1[j*7+5] = newVal[1];
        buf1[j*7+6] = newVal[2];
        j++;
      }
      if(corner == 2) {
        //cout << seed << "  " << newVal <<  endl;
        buf2[j*7] = (x >> 8) & 0xFF;
        buf2[j*7+1] = x & 0xFF;
        buf2[j*7+2] = (y >> 8) & 0xFF;
        buf2[j*7+3] = y & 0xFF;
        buf2[j*7+4] = newVal[0];
        buf2[j*7+5] = newVal[1];
        buf2[j*7+6] = newVal[2];
        j++;
      }
      else if(corner == 3) {
        //cout << seed << "  " << newVal <<  endl;
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
    buf0Length = j;
  }
  if(corner == 1) {
    buf1Length = j;
  }
  if(corner == 2) {
    buf2Length = j;
    //for(int i=0;i<buf2Length*7+7;i++) cout << (int)buf2[i] << " ";
  }
  else if(corner == 3) {
    bufLength = j;
  }
}

void FindColoursThread0(int corner, int sobolPoints)
{
  int offset_cols = 0;
  int offset_rows = 0;
  while(!finished)
  {
    unique_lock<mutex> lk(m);
    conVar.wait(lk, []{return imageReady0;});
    //cout << "running" << corner << endl;
    FindColours(corner, offset_cols, offset_rows, sobolPoints);
    imageReady0 = 0;
    lk.unlock();
    conVar.notify_one();
  }
}
  
void FindColoursThread1(int corner, int sobolPoints)
{
  int offset_cols = tmp_frame.cols/2;
  int offset_rows = 0;
  while(!finished)
  {
    unique_lock<mutex> lk(m);
    conVar1.wait(lk, []{return imageReady1;});
    //cout << "running" << corner <<  endl;
    FindColours(corner, offset_cols, offset_rows, sobolPoints);
    imageReady1 = 0;
    lk.unlock();
    conVar1.notify_one();
  }
}

void FindColoursThread2(int corner, int sobolPoints)
{
  int offset_cols = 0;
  int offset_rows = tmp_frame.rows/2;
  while(!finished)
  {
    unique_lock<mutex> lk(m);
    conVar2.wait(lk, []{return imageReady2;});
    //cout << "running" << corner <<  endl;
    FindColours(corner, offset_cols, offset_rows, sobolPoints);
    imageReady2 = 0;
    lk.unlock();
    conVar2.notify_one();
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
  //cvtColor(dst, dst, CV_GRAY2RGB);
  dst_out = dst.clone(); 
  //cout << "edges\n";
  {
    lock_guard<mutex> lk(m);
    imageReady0 = 1;
  }
  conVar.notify_one();
  {
    lock_guard<mutex> lk(m);
    imageReady1 = 1;
  }
  conVar1.notify_one();
  {
    lock_guard<mutex> lk(m);
    imageReady2 = 1;
  }
  conVar2.notify_one();
  FindColours(3, dst.cols/2, dst.rows/2, POINTSPERTHREAD);
  //cout << "running3\n";
  {
    unique_lock<mutex> lk(m);
    conVar.wait(lk, []{return !imageReady0;});
  }
  {
    unique_lock<mutex> lk(m);
    conVar1.wait(lk, []{return !imageReady1;});
  }
  {
    unique_lock<mutex> lk(m);
    conVar2.wait(lk, []{return !imageReady2;});
  }
  //cout << "sending\n";
  for(int i=0;i<buf0Length*7;i++) buf[bufLength*7 + i] = buf0[i];
  for(int i=0;i<buf1Length*7;i++) buf[bufLength*7 + buf0Length*7 + i] = buf1[i];
  //for(int i=0;i<buf0Length*7+bufLength*7+buf1Length*7+14;i++) cout << (int)buf[i] << " ";
  for(int i=0;i<buf2Length*7;i++) buf[bufLength*7 + buf0Length*7 + buf1Length*7 + i] = buf2[i];
  for(int i=0;i<7;i++) buf[bufLength*7 + buf0Length*7 + buf1Length*7 + buf2Length*7 + i] = 0;
  resize(dst_out, dst_out, Size(), 0.5, 0.5, CV_INTER_AREA);
  imencode(".png", dst_out, dstBuf);
  //cout << bufLength*7 << "  " << buf1Length*7 << "  " << dstBuf.size() << endl;
  //cout << "2\n";
  for(int i=0;i<dstBuf.size();i++) buf[bufLength*7 + buf0Length*7 + buf1Length*7 + buf2Length*7 + 7 + i] = dstBuf[i];
  //for(int i=0;i<(buf1Length+bufLength+buf0Length+buf2Length+2);i++) {
  //   for(int a=0;a<7;a++) cout << (int)buf[i*7+a] << " ";
  //   cout << endl;
  //}
  //cout << "done\n";
  bufSize = bufLength*7 + buf0Length*7 + buf1Length*7 + buf2Length*7 + 7 + dstBuf.size();
  //imshow( "Edge Map", dst );
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
  if (inet_aton(argv[1], &remaddr.sin_addr)==0) {
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
  cap.set(CV_CAP_PROP_FPS, FRAMERATE);
  int FPS = cap.get(CV_CAP_PROP_FPS);
  cout << "FPS Limit: " << FPS << endl;
  cout << "Low Threshold: " << lowThreshold << endl;
  cout << "Points per thread: " << POINTSPERTHREAD << endl;
  cap >> tmp_frame;
  if(tmp_frame.empty())
  {
      printf("can not read data from the video source\n");
      return -1;
  }

  //namedWindow( "Edge Map", 1 );
  //cvMoveWindow( "Edge Map", 0, 40 );
  namedWindow("Camera", 1);
  //cvMoveWindow( "Camera", tmp_frame.cols + 20, 40 );
  //createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
  //createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, 0 );

  t[0] = thread(FindColoursThread0, 0, POINTSPERTHREAD);
  t[1] = thread(FindColoursThread1, 1, POINTSPERTHREAD);
  t[2] = thread(FindColoursThread2, 2, POINTSPERTHREAD);

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
  for(int i=0;i<3;i++) t[i].join();
  //t.join();
  return 0;
  }
