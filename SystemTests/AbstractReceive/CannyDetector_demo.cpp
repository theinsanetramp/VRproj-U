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
#include <mutex>
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
mutex showm;
vector<Mat> showBuf;

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
uchar controlBuf[6];

vector<uchar> imageBuf;
vector<uchar> image2Buf;

void SendControl()
{
  int j;
  int X[2], Y[2];
  int threshUp = 0;
  int threshDown = 0;
  uchar lowThreshold = 20;
  for(int i=0;i<6;i++) controlBuf[i] = 0;
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
          if(j == 12) {
            controlBuf[5] = (1+controlBuf[5])%2;
            cout << "Drive mode = " << (int)controlBuf[5] << endl;
          }
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
        if(j == 1) {
          //cout << j << " stick with coords: " << X[j] << " " << Y[j] << endl;
          controlBuf[3] = X[j]/258;
          controlBuf[4] = Y[j]/258;
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
    sendto(fd, controlBuf, 6, 0, (struct sockaddr *)&remaddr, addrlen);
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
    if(i>200) {
      cout << "Colours-Image1 splitter not found\n";
      seedList.clear();
      imageBuf.clear();
      image2Buf.clear();
      return;
    }
  } 
  while(!(buf[i*7+0] == 0 && buf[i*7+1] == 1 
    && buf[i*7+2] == 2 && buf[i*7+3] == 3 
    && buf[i*7+4] == 4 && buf[i*7+5] == 5
    && buf[i*7+6] == 6));
  i++;
  //for(int j=i*7;j<recvlen;j++) imageBuf.push_back(buf[j]); 
  i = i*7;
  do
  {
    imageBuf.push_back(buf[i]); 
    i++;
    if(i>10000) {
      cout << "Image1-Image2 splitter not found\n";
      seedList.clear();
      imageBuf.clear();
      image2Buf.clear();
      return;
    }
  } 
  while(!(buf[i] == 0 && buf[i+1] == 1 
    && buf[i+2] == 2 && buf[i+3] == 3 
    && buf[i+4] == 4 && buf[i+5] == 5
    && buf[i+6] == 6));
  //for(int j=i-7;j<i+14;j++) cout << (int)buf[j] << endl;
  i += 7;
  do
  {
    image2Buf.push_back(buf[i]); 
    i++;
  } 
  while(i != recvlen);
  receivedImage = imdecode(imageBuf, IMREAD_COLOR);
  //cout << imageBuf.size() << endl;
  if(receivedImage.empty()) {
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
  //erode( receivedImage, receivedImage, element );
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
  if(receivedImage2.empty()) {
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
  //erode( receivedImage2, receivedImage2, element );
  seedList.clear();
  imageBuf.clear();
  image2Buf.clear();
  showm.lock();
  showBuf.push_back(receivedImage);
  showBuf.push_back(receivedImage2);
  showm.unlock();
}

void UDPReceive()
{
  this_thread::sleep_for(chrono::milliseconds(500));
  recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
  printf("received %d bytes\n", recvlen);
  addressReceived = 1;
  if (recvlen > 0) ReceivePoints();
  cvMoveWindow( "Edge Map 2", receivedImage.cols + 70, 40 );
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
  receivedImage.create(200,200,CV_8UC3);
  receivedImage2.create(200,200,CV_8UC3);

  Mat leftDisplay, rightDisplay;

  /// Wait until user exit program by pressing a key
  int k;
  do{
    showm.lock();
    if(showBuf.size() > 0) {
      rightDisplay = showBuf.back();
      showBuf.pop_back();
      leftDisplay = showBuf.back();
      showBuf.pop_back();
      imshow( "Edge Map", leftDisplay ); 
      imshow( "Edge Map 2", rightDisplay ); 
    }
    showm.unlock();
  	/// Wait until user exit program by pressing a key
  	k = waitKey(1);
  }
  while(k != 27);
  finished = 1;
  shutdown(fd, SHUT_RDWR);
  close(fd);
  t.join();
  control.join();
  //imwrite("LGimble.jpg", receivedImage);
  //imwrite("RGimble.jpg", receivedImage2);
  return 0;
}