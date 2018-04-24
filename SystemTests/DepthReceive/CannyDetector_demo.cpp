#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
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
using namespace cv::ximgproc;
using namespace std;

#define SERVICE_PORT  21234
#define BUFSIZE 40960

Mat receivedImage;
Mat receivedImage2;
Mat left_for_matcher, right_for_matcher;
Mat filtered_disp_vis;
Mat left_disp,right_disp;
Mat filtered_disp;

thread t;
thread control;
int finished = 0;
int addressReceived = 0;
mutex showm;
vector<Mat> showBuf;

struct sockaddr_in myaddr;  /* our address */
struct sockaddr_in remaddr; /* remote address */
socklen_t addrlen = sizeof(remaddr);    /* length of addresses */
int recvlen;      /* # bytes received */
int fd;       /* our socket */
uchar buf[BUFSIZE]; /* receive buffer */
uchar controlBuf[6];

vector<uchar> imageBuf;
vector<uchar> image2Buf;

Mat R1, R2, P1, P2, Q;
Mat K1, K2, R;
Vec3d T;
Mat D1, D2;
Mat lmapx, lmapy, rmapx, rmapy;

int wsize = 15;
int max_disp = 64;
double lambda = 16000;
double sigma = 1.2;
double vis_mult = 1;
Ptr<DisparityWLSFilter> wls_filter;

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

void ReceivePoints()
{
  //Floodfill from quasi-random points
  int i = 0;
  do
  {
    imageBuf.push_back(buf[i]); 
    i++;
    if(i>40000) {
      cout << "Image1-Image2 splitter not found\n";
      imageBuf.clear();
      image2Buf.clear();
      return;
    }
  } 
  while(!(buf[i] == 0 && buf[i+1] == 0 
    && buf[i+2] == 0 && buf[i+3] == 0 
    && buf[i+4] == 0 && buf[i+5] == 0
    && buf[i+6] == 0 && buf[i+7] == 0
    && buf[i+8] == 0 && buf[i+9] == 0
    && buf[i+10] == 0 && buf[i+11] == 0
    && buf[i+12] == 0 && buf[i+13] == 0));
  //for(int j=i-7;j<i+14;j++) cout << (int)buf[j] << endl;
  i += 14;
  do
  {
    image2Buf.push_back(buf[i]); 
    i++;
  } 
  while(!(buf[i] == 0 && buf[i+1] == 0 
    && buf[i+2] == 0 && buf[i+3] == 0 
    && buf[i+4] == 0 && buf[i+5] == 0
    && buf[i+6] == 0 && buf[i+7] == 0
    && buf[i+8] == 0 && buf[i+9] == 0
    && buf[i+10] == 0 && buf[i+11] == 0
    && buf[i+12] == 0 && buf[i+13] == 0));
  receivedImage = imdecode(imageBuf, CV_LOAD_IMAGE_COLOR);
  //cout << image2Buf.size() << endl;
  if(receivedImage.empty()) {
    cout << "Images not received\n";
    imageBuf.clear();
    image2Buf.clear();
    return;
  }
  resize(receivedImage, receivedImage, Size(), 2, 2, CV_INTER_CUBIC);
  receivedImage2 = imdecode(image2Buf, CV_LOAD_IMAGE_COLOR);
  if(receivedImage2.empty()) {
    cout << "Only one image received\n";
    imageBuf.clear();
    image2Buf.clear();
    return;
  }
  resize(receivedImage2, receivedImage2, Size(), 2, 2, CV_INTER_CUBIC);

  initUndistortRectifyMap(K1, D1, R1, P1, receivedImage.size(), CV_32F, lmapx, lmapy);
  initUndistortRectifyMap(K2, D2, R2, P2, receivedImage2.size(), CV_32F, rmapx, rmapy);
  remap(receivedImage, receivedImage, lmapx, lmapy, cv::INTER_LINEAR);
  remap(receivedImage2, receivedImage2, rmapx, rmapy, cv::INTER_LINEAR);

  left_for_matcher  = receivedImage.clone();
  right_for_matcher = receivedImage2.clone();

  Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
  wls_filter = createDisparityWLSFilter(left_matcher);
  Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

  cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
  cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);
  left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
  right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
  wls_filter->setLambda(lambda);
  wls_filter->setSigmaColor(sigma);
  wls_filter->filter(left_disp,left_for_matcher,filtered_disp,right_disp);
  getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);

  imageBuf.clear();
  image2Buf.clear();
  showm.lock();
  showBuf.push_back(receivedImage);
  showBuf.push_back(receivedImage2);
  showBuf.push_back(filtered_disp_vis);
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
  cvMoveWindow( "Depth", 0, receivedImage.rows + 70 );
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

  FileStorage fs1("cam_stereo.yml", cv::FileStorage::READ);
  fs1["K1"] >> K1;
  fs1["K2"] >> K2;
  fs1["D1"] >> D1;
  fs1["D2"] >> D2;
  fs1["R"] >> R;
  fs1["T"] >> T;

  fs1["R1"] >> R1;
  fs1["R2"] >> R2;
  fs1["P1"] >> P1;
  fs1["P2"] >> P2;
  fs1["Q"] >> Q;

  GamepadInit();

  t = thread(UDPReceive);
  control = thread(SendControl);

  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map", 0, 40 ); 
  namedWindow("Edge Map 2", 1);
  cvMoveWindow( "Edge Map 2",  500, 40 );
  namedWindow("Depth", 1);
  cvMoveWindow( "Depth",  40, 500 );
  receivedImage.create(200,200,CV_8UC3);
  receivedImage2.create(200,200,CV_8UC3);

  Mat leftDisplay, rightDisplay, disp;

  /// Wait until user exit program by pressing a key
  int k;
  do{
    showm.lock();
    if(showBuf.size() > 0) {
      disp = showBuf.back();
      showBuf.pop_back();
      rightDisplay = showBuf.back();
      showBuf.pop_back();
      leftDisplay = showBuf.back();
      showBuf.pop_back();
      imshow( "Edge Map", leftDisplay ); 
      imshow( "Edge Map 2", rightDisplay ); 
      imshow( "Depth", disp ); 
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
  //imwrite("jpgtest.jpg", disp);
  //imwrite("RGimble.jpg", receivedImage2);
  return 0;
}