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
#define BUFLEN 4096

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

struct sockaddr_in myaddr, remaddr;
int fd, j, k, slen=sizeof(remaddr);
string server = "127.0.0.1"; /* change this to use a different server */
int buf[BUFLEN];

void CannyThreshold(int, void*)
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Reduce noise with a kernel 5x5
  blur( src_gray, detected_edges, Size(3,3) );
  //imwrite("buildBlur.jpg",detected_edges);
  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
  //imwrite("buildCanny.jpg",detected_edges);
  findContours( detected_edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
  /// Using Canny's output as a mask, we display our result
  // dst = Scalar::all(0);
  // Mat element = getStructuringElement( dilation_type,
  //                                      Size( 2*dilation_size + 1, 2*dilation_size+1 ),
  //                                      Point( dilation_size, dilation_size ) );

  // /// Apply the dilation operation
  // dilate( detected_edges, dst, element );
  // //imwrite("buildDilated.jpg",dst);
  // cvtColor(dst, dst, CV_GRAY2RGB);

  Mat dst = Mat::zeros( detected_edges.size(), CV_8UC3 );
  Scalar color = Scalar(255,255,255);
  for( size_t i = 0; i< contours.size(); i++ )
     {
       drawContours( dst, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
     }
  int contoursSize = 0;
  for(int i = 0; i < contours.size(); i++)
  {
    contoursSize += contours[i].size();
  }
  cout << contoursSize << endl;

  //Floodfill from quasi-random points
  j = 0;
  for (unsigned long long i = 0; i < 600; ++i)
    {
        int x = src.cols * sobol::sample(i, 0);
        int y = src.rows * sobol::sample(i, 1);
        Point seed = Point(x, y);
        if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
        {
          flood_mask = 0;
          Rect ccomp;
          floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
                  Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
          Scalar newVal = mean(src,flood_mask);
          buf[j*5] = x;
          buf[j*5+1] = y;
          buf[j*5+2] = newVal[0];
          buf[j*5+3] = newVal[1];
          buf[j*5+4] = newVal[2];
          floodFill(dst, /*circle_mask,*/ seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                  Scalar(upDiff, upDiff, upDiff), flags);
          j++;
        }
    }
  cout << j << endl;
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
  //if (inet_aton(server, &remaddr.sin_addr)==0) {
  //  fprintf(stderr, "inet_aton() failed\n");
  //  exit(1);
  //}

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
  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  cout << "Sending packet to " << server << " port " << SERVICE_PORT << endl;
  // for(int j=0;j<10;j++) {
  //   cout << buf[j] << endl;
  // }
  cout << j*20 << endl;
  if (sendto(fd, buf, j*20, 0, (struct sockaddr *)&remaddr, slen)==-1)
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