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
#include <thread>
#include <chrono>
#include <unistd.h>
#include "gamepad.h"

using namespace cv;
using namespace std;

#define SERVICE_PORT  21234
#define BUFSIZE 40960

Mat receivedImage;
Mat receivedImage2;

thread t;
thread control;
int finished = 0;
int addressReceived = 0;

struct SeedData
{
  Point seed;
  Scalar colour;
};
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

vector<SeedData> seedList;

struct sockaddr_in myaddr;  /* our address */
struct sockaddr_in remaddr; /* remote address */
socklen_t addrlen = sizeof(remaddr);    /* length of addresses */
int recvlen;      /* # bytes received */
int fd;       /* our socket */
uchar buf[BUFSIZE]; /* receive buffer */
uchar controlBuf[3];

vector<uchar> imageBuf;
vector<uchar> image2Buf;

void SendControl()
{
  int j;
  int X[2], Y[2];
  int threshUp = 0;
  int threshDown = 0;
  uchar lowThreshold = 20;
  while(!addressReceived);
  while(!finished)
  {
    GamepadUpdate();
    if (GamepadIsConnected(GAMEPAD_0)) {
      for (j = 0; j != BUTTON_COUNT; ++j) {
        if (GamepadButtonTriggered(GAMEPAD_0, (GAMEPAD_BUTTON)j)) {
          //cout << j << " pressed\n";
          if(j == 8) threshDown = 1;
          if(j == 9) threshUp = 1;
        } 
        else if (GamepadButtonReleased(GAMEPAD_0, (GAMEPAD_BUTTON)j)) {
          if(j == 8) threshDown = 0;
          if(j == 9) threshUp = 0;
        }
      }
      for (j = 0; j != TRIGGER_COUNT; ++j) {
        if (GamepadTriggerTriggered(GAMEPAD_0, (GAMEPAD_TRIGGER)j)) {
          //cout << j << " trigger pressed\n";
        }
      }
      for (j = 0; j != STICK_COUNT; ++j) {
        GamepadStickXY(GAMEPAD_0, (GAMEPAD_STICK)j, &X[j], &Y[j]);
        if(j == 0) {
          //cout << j << " stick with coords: " << X[j] << " " << Y[j] << endl;
          controlBuf[0] = X[j]/258;
          controlBuf[1] = Y[j]/258;
        }
      }
    }
    if(threshDown && !threshUp) {
      lowThreshold--;
      cout << "Next low threshold: " << (int)lowThreshold << endl;
    }
    if(threshUp && !threshDown) {
      lowThreshold++;
      cout << "Next low threshold: " << (int)lowThreshold << endl;
    }
    controlBuf[2] = lowThreshold;
    //for(int i=0;i<4;i++) cout << (int)controlBuf[i] << " ";
    //cout << endl;
    sendto(fd, controlBuf, 3, 0, (struct sockaddr *)&remaddr, addrlen);
    this_thread::sleep_for(chrono::milliseconds(50));
  }
}

SeedData MakeSeedData(Point seed, Scalar colour)
{
  SeedData s;
  s.seed = seed;
  s.colour = colour;
  return s;
}

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
    //cout << seed << "  " << newVal << endl;
    seedList.push_back(MakeSeedData(seed, newVal));
    i++;
  } 
  while(!(buf[i*7+0] == 0 && buf[i*7+1] == 0 
    && buf[i*7+2] == 0 && buf[i*7+3] == 0 
    && buf[i*7+4] == 0 && buf[i*7+5] == 0
    && buf[i*7+6] == 0));
  i++;
  //for(int j=i*7;j<recvlen;j++) imageBuf.push_back(buf[j]); 
  i = i*7;
  do
  {
    imageBuf.push_back(buf[i]); 
    i++;
  } 
  while(!(buf[i] == 0 && buf[i+1] == 0 
    && buf[i+2] == 0 && buf[i+3] == 0 
    && buf[i+4] == 0 && buf[i+5] == 0
    && buf[i+6] == 0));
  //for(int j=i-7;j<i+14;j++) cout << (int)buf[j] << endl;
  i += 7;
  do
  {
    image2Buf.push_back(buf[i]); 
    i++;
  } 
  while(!(buf[i] == 0 && buf[i+1] == 0 
    && buf[i+2] == 0 && buf[i+3] == 0 
    && buf[i+4] == 0 && buf[i+5] == 0
    && buf[i+6] == 0));
  receivedImage = imdecode(imageBuf, IMREAD_COLOR);
  //cout << imageBuf.size() << endl;
  if(imageBuf.size() < 10) {
    cout << "Images not received\n";
    seedList.clear();
    imageBuf.clear();
    image2Buf.clear();
    return;
  }
  resize(receivedImage, receivedImage, Size(), 2, 2, CV_INTER_CUBIC);
  for(int i=0;i<receivedImage.rows;i++) {
    for(int j=0;j<receivedImage.cols;j++) {
      if(receivedImage.at<Vec3b>(i,j)[0] < 170) receivedImage.at<Vec3b>(i,j) = Vec3b(0,0,0);
      else receivedImage.at<Vec3b>(i,j) = Vec3b(255,255,255);
    }
  }
  erode( receivedImage, receivedImage, element );
  //cout << seedList.size() << endl;
  for(int j=0;j<seedList.size();j++)
  {
    seedList[j].seed.x = seedList[j].seed.x*2;
    seedList[j].seed.y = seedList[j].seed.y*2;
    if(receivedImage.at<Vec3b>(seedList[j].seed)[0] == 0 && 
      receivedImage.at<Vec3b>(seedList[j].seed)[1] == 0 && 
      receivedImage.at<Vec3b>(seedList[j].seed)[2] == 0)
    { 
      floodFill(receivedImage, seedList[j].seed, seedList[j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
          Scalar(upDiff, upDiff, upDiff), flags);
    }
    //circle(receivedImage, seedList[j].seed, 5, 255, -1);
  }
  for(int i=0;i<receivedImage.rows;i++) {
    for(int j=0;j<receivedImage.cols;j++) {
      if(receivedImage.at<Vec3b>(i,j)[0] == 0) receivedImage.at<Vec3b>(i,j) = Vec3b(255,0,255);
    }
  }
  receivedImage2 = imdecode(image2Buf, IMREAD_COLOR);
  if(image2Buf.size() < 10) {
    cout << "Only one image received\n";
    seedList.clear();
    imageBuf.clear();
    image2Buf.clear();
    return;
  }
  resize(receivedImage2, receivedImage2, Size(), 2, 2, CV_INTER_CUBIC);
  for(int i=0;i<receivedImage2.rows;i++) {
    for(int j=0;j<receivedImage2.cols;j++) {
      if(receivedImage2.at<Vec3b>(i,j)[0] < 170) receivedImage2.at<Vec3b>(i,j) = Vec3b(0,0,0);
      else receivedImage2.at<Vec3b>(i,j) = Vec3b(255,255,255);
    }
  }
  erode( receivedImage2, receivedImage2, element );
  seedList.clear();
  imageBuf.clear();
  image2Buf.clear();
  if(!finished) {
    imshow( "Edge Map", receivedImage ); 
    imshow( "Edge Map 2", receivedImage2 ); 
  }
}

void UDPReceive()
{
  this_thread::sleep_for(chrono::milliseconds(500));
  recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
  printf("received %d bytes\n", recvlen);
  addressReceived = 1;
  if (recvlen > 0) ReceivePoints();
  while(!finished)
  {
    //printf("waiting on port %d\n", SERVICE_PORT);
    //if(finished) return;
    recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);

    printf("received %d bytes\n", recvlen);
    if (recvlen > 0) {
      //buf[recvlen] = 0;
      //for(int i=0;i<10;i++) printf("%i\n", buf[i]);
      ReceivePoints();
    }
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

  GamepadInit();

  t = thread(UDPReceive);
  control = thread(SendControl);

  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map", 0, 40 ); 
  namedWindow("Edge Map 2", 1);
  cvMoveWindow( "Edge Map 2",  500, 40 );

  /// Wait until user exit program by pressing a key
  int k;
  do{
  	 /// Wait until user exit program by pressing a key
  	 k = waitKey(0);
  }
  while(k != 27);
  finished = 1;
  shutdown(fd, SHUT_RDWR);
  close(fd);
  t.join();
  control.join();
  return 0;
}