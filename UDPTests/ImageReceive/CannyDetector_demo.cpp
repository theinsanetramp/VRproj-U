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
#define BUFSIZE 4096

Mat src, src_gray;
Mat dst, detected_edges;
Mat flood_mask;

//Canny variables
int lowThreshold = 50;
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

struct sockaddr_in myaddr;  /* our address */
struct sockaddr_in remaddr; /* remote address */
socklen_t addrlen = sizeof(remaddr);    /* length of addresses */
int recvlen;      /* # bytes received */
int fd;       /* our socket */
int buf[BUFSIZE]; /* receive buffer */

void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 5x5
  blur( src_gray, detected_edges, Size(3,3) );
  //imwrite("buildBlur.jpg",detected_edges);
  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  //imwrite("buildCanny.jpg",detected_edges);
  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);
  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( detected_edges, dst, element );
  //imwrite("buildDilated.jpg",dst);
  cvtColor(dst, dst, CV_GRAY2RGB); 
 }

void ReceivePoints()
{
  //Floodfill from quasi-random points
  for (unsigned long long i = 0; i < recvlen/20; ++i)
    {
        int x = buf[i*5];
        int y = buf[i*5+1];
        Point seed = Point(x, y);
        Rect ccomp;
        Scalar newVal = Scalar(buf[i*5+2], buf[i*5+3], buf[i*5+4]);
        floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                Scalar(upDiff, upDiff, upDiff), flags);
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
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );
  flood_mask.create(src.rows+2, src.cols+2, CV_8UC1);

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );
  //imwrite("buildGray.jpg",src_gray);

  /// Create a Trackbar for user to enter threshold
  //createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

  CannyThreshold(0, 0);

  
  printf("waiting on port %d\n", SERVICE_PORT);
  recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
  printf("received %d bytes\n", recvlen);
  if (recvlen > 0) {
    //buf[recvlen] = 0;
    //for(int i=0;i<10;i++) printf("%i\n", buf[i]);
    ReceivePoints();
  }

  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  imshow( "Edge Map", dst );

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