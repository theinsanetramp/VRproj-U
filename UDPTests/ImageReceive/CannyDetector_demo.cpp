#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "sobol.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <netdb.h>
#include <sys/socket.h>

using namespace cv;
using namespace std;

#define SERVICE_PORT  21234
#define BUFSIZE 40960

Mat receivedImage;

struct SeedData
{
  Point seed;
  Scalar colour;
};

vector<SeedData> seedList;

//Canny variables
int lowThreshold = 25;
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
//dilation variables
int dilation_type = MORPH_RECT;
int dilation_size = 1;
Mat element = getStructuringElement( dilation_type,
                                     Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                     Point( dilation_size, dilation_size ) );

struct sockaddr_in myaddr;  /* our address */
struct sockaddr_in remaddr; /* remote address */
socklen_t addrlen = sizeof(remaddr);    /* length of addresses */
int recvlen;      /* # bytes received */
int fd;       /* our socket */
uchar buf[BUFSIZE]; /* receive buffer */

vector<uchar> imageBuf;

SeedData MakeSeedData(Point seed, Scalar colour)
{
  SeedData s;
  s.seed = seed;
  s.colour = colour;
  return s;
}

// void CannyThreshold(int, void*)
// {
//   /// Reduce noise with a kernel 5x5
//   blur( dst, dst, Size(5,5) );
//   /// Canny detector
//   Canny( dst, dst, lowThreshold, lowThreshold*ratio, kernel_size );
//   /// Apply the dilation operation
//   dilate( dst, dst, element );
//   //imwrite("buildDilated.jpg",dst);
//   cvtColor(dst, dst, CV_GRAY2RGB); 
//  }

void ReceivePoints()
{
  //Floodfill from quasi-random points
  int i = 0;
  do
  {
    int x = (buf[i*7] << 8) + buf[i*7+1];
    int y = (buf[i*7+2] << 8) + buf[i*7+3];
    Point seed = Point(x, y);
    Scalar newVal = Scalar(buf[i*7+4], buf[i*7+5], buf[i*7+6]);
    cout << seed << "  " << newVal << endl;
    seedList.push_back(MakeSeedData(seed, newVal));
    i++;
  } 
  while(!(buf[i*7] == 0 && buf[i*7+2] == 0 && buf[i*7+4] == 0 && buf[i*7+5] == 0 && buf[i*7+6] == 0));
  i++;
  for(int j=i*7;j<recvlen;j++) imageBuf.push_back(buf[j]);
  receivedImage = imdecode(imageBuf, IMREAD_COLOR);
  resize(receivedImage, receivedImage, Size(), 2, 2, CV_INTER_CUBIC);
  for(int i=0;i<receivedImage.rows;i++) {
    for(int j=0;j<receivedImage.cols;j++) {
      if(receivedImage.at<Vec3b>(i,j)[0] < 170) receivedImage.at<Vec3b>(i,j) = Vec3b(0,0,0);
      else receivedImage.at<Vec3b>(i,j) = Vec3b(255,255,255);
    }
  }
  for(int j=0;j<seedList.size();j++)
  {
    if(receivedImage.at<Vec3b>(seedList[j].seed)[0] == 0 && 
      receivedImage.at<Vec3b>(seedList[j].seed)[1] == 0 && 
      receivedImage.at<Vec3b>(seedList[j].seed)[2] == 0)
    { 
      floodFill(receivedImage, seedList[j].seed, seedList[j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
          Scalar(upDiff, upDiff, upDiff), flags);
    }
    //else circle(receivedImage, seedList[j].seed, 5, 255, -1);
  }
}

/** @function main */
int main( int argc, char** argv )
{
  /* create a UDP socket */

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("cannot create socket\n");
    return 0;
  }

  /* bind the socket to any valid IP address and a specific port */

  memset((char *)&myaddr, 0, sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  myaddr.sin_port = htons(SERVICE_PORT);

  if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
    perror("bind failed");
    return 0;
  }

  /// Load an image
  // src = imread( argv[1] );

  // if( !src.data )
  // { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  // dst.create( src.size(), src.type() );
  // flood_mask.create(src.rows+2, src.cols+2, CV_8UC1);

  /// Convert the image to grayscale
  // cvtColor( src, dst, CV_BGR2GRAY );
  //imwrite("buildGray.jpg",src_gray);

  /// Create a Trackbar for user to enter threshold
  //createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

  // CannyThreshold(0, 0);

  printf("waiting on port %d\n", SERVICE_PORT);
  recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
  printf("received %d bytes\n", recvlen);
  if (recvlen > 0) {
    //buf[recvlen] = 0;
    //for(int i=0;i<10;i++) printf("%i\n", buf[i]);
    ReceivePoints();
  }

  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map", 0, 40 );
  imshow( "Edge Map", receivedImage );  

  /// Wait until user exit program by pressing a key
  int k;
  do{
  	 /// Wait until user exit program by pressing a key
  	 k = waitKey(0);
  }
  while(k != 27);
  //imwrite("Final.jpg",dst);
  return 0;
  }