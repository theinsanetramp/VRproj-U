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
#include <arpa/inet.h>

using namespace cv;
using namespace std;

#define SERVICE_PORT  21234
#define BUFLEN 40960

Mat src, dst_out;
Mat dst, flood_mask;
Mat mean_mask;

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

struct sockaddr_in myaddr, remaddr;
int fd, bufSize, k, slen=sizeof(remaddr);
char server[] = "10.9.177.131"; /* change this to use a different server */
uchar buf[BUFLEN];

void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 5x5
  blur( dst, dst, Size(5,5) );
  /// Canny detector
  Canny( dst, dst, lowThreshold, lowThreshold*ratio, kernel_size );
  /// Apply the dilation operation
  dilate( dst, dst, element );
  dst_out = dst.clone();
  cvtColor(dst, dst, CV_GRAY2RGB);
  //Floodfill from quasi-random points
  int j = 0;
  for (unsigned long long i = 0; i < 600; ++i)
  {
    int x = src.cols * sobol::sample(i, 0);
    int y = src.rows * sobol::sample(i, 1);
    Point seed = Point(x, y);
    if(dst.at<Vec3b>(seed)[0] == 0)
    {
      flood_mask = 0;
      floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
              Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8));
      //cout << "before\n";
      for(int i=0;i<mean_mask.rows-2;i++)
      {
        for(int a=0;a<mean_mask.cols;a++)
        {
          mean_mask.at<int>(i,a) = flood_mask.at<int>(i+1,a+1);
        }
      }
      //cout << "aadf\n";
      Scalar newVal = mean(src,mean_mask);
      buf[j*7] = (x >> 8) & 0xFF;
      buf[j*7+1] = x & 0xFF;
      buf[j*7+2] = (y >> 8) & 0xFF;
      buf[j*7+3] = y & 0xFF;
      buf[j*7+4] = newVal[0];
      buf[j*7+5] = newVal[1];
      buf[j*7+6] = newVal[2];
      //cout << seed << "  " << newVal << endl;
      //floodFill(dst, seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
      //        Scalar(upDiff, upDiff, upDiff), flags);
      j++;
    }
  }
  for(int i=0;i<7;i++) buf[j*7+i] = 0;
  //cout << j*7 << endl;
  j++;
  vector<uchar> dstBuf;
  resize(dst_out, dst_out, Size(), 0.5, 0.5, CV_INTER_AREA);
  imencode(".png", dst_out, dstBuf);
  //cout << dstBuf.size() << endl;
  bufSize = j*7 + dstBuf.size();
  for(int i=0;i<dstBuf.size();i++) buf[j*7 + i] = dstBuf[i];

  imshow( "Edge Map", dst );
 }


/** @function main */
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

  /// Load an image
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );
  flood_mask.create(src.rows+2, src.cols+2, CV_8UC1);
  mean_mask.create(src.size(), CV_8UC1);

  /// Convert the image to grayscale
  cvtColor( src, dst, CV_BGR2GRAY );
  //imwrite("buildGray.jpg",src_gray);
  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );

  /// Create a Trackbar for user to enter threshold
  //createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  cout << "Sending packet to " << server << " port " << SERVICE_PORT << endl;
  // for(int j=0;j<10;j++) {
  //   cout << buf[j] << endl;
  // }
  cout << bufSize << endl;
  if (sendto(fd, buf, bufSize, 0, (struct sockaddr *)&remaddr, slen)==-1)
    perror("sendto");

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
