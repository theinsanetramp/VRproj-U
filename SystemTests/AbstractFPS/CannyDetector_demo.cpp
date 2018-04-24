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
Mat colouredImage;
Mat left_for_matcher, right_for_matcher;
Mat filtered_disp_vis, last_vis, consist_vis;
Mat left_disp,right_disp;
Mat filtered_disp;
Mat weighted_map, output_map;

thread t;
thread control;
int finished = 0;
int addressReceived = 0;
mutex showm;
vector<int> msglenbuf;
uchar messageBuf[BUFSIZE];

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
uchar recvBuf[BUFSIZE];
uchar buf[BUFSIZE];
int msglen;
uchar controlBuf[6];

vector<uchar> imageBuf;
vector<uchar> image2Buf;

Mat R1, R2, P1, P2, Q;
Mat K1, K2, R;
Vec3d T;
Mat D1, D2;
Mat lmapx, lmapy, rmapx, rmapy;

int wsize = 15;
int max_disp = 32;
double lambda = 1000;
double sigma = 3.5;
double vis_mult = 3;
Ptr<DisparityWLSFilter> wls_filter;
int first = 1;
int frames = 0;

void SendControl()
{
  int j;
  int X[2], Y[2];
  int threshUp = 0;
  int threshDown = 0;
  uchar lowThreshold = 30;
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
      if(threshDown && !threshUp) {
        lowThreshold--;
        cout << "Next low threshold: " << (int)lowThreshold << endl;
      }
      if(threshUp && !threshDown) {
        lowThreshold++;
        cout << "Next low threshold: " << (int)lowThreshold << endl;
      }
    }
    else {
      for(int i=0;i<6;i++) controlBuf[i] = 0;
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
  while(i != msglen);
  receivedImage = imdecode(imageBuf, IMREAD_COLOR);
  //cout << imageBuf.size() << endl;
  if(receivedImage.empty()) {
    cout << "Images not received\n";
    seedList.clear();
    imageBuf.clear();
    image2Buf.clear();
    return;
  }
  receivedImage2 = imdecode(image2Buf, IMREAD_COLOR);
  if(receivedImage2.empty()) {
    cout << "Only one image received\n";
    seedList.clear();
    imageBuf.clear();
    image2Buf.clear();
    return;
  }

  colouredImage = receivedImage.clone();
  dilate( colouredImage, colouredImage, element );
  //erode( colouredImage, colouredImage, element );
  // for(int i=0;i<colouredImage.rows;i++) {
  //   for(int j=0;j<colouredImage.cols;j++) {
  //     if(colouredImage.at<Vec3b>(i,j)[0] < 170) colouredImage.at<Vec3b>(i,j) = Vec3b(0,0,0);
  //     else colouredImage.at<Vec3b>(i,j) = Vec3b(255,255,255);
  //   }
  // }
  //cout << seedList.size() << endl;
  for(int j=0;j<seedList.size();j++)
  {
    //seedList[j].seed.x = seedList[j].seed.x*2;
    //seedList[j].seed.y = seedList[j].seed.y*2;
    if(colouredImage.at<Vec3b>(seedList[j].seed)[0] == 0 && 
      colouredImage.at<Vec3b>(seedList[j].seed)[1] == 0 && 
      colouredImage.at<Vec3b>(seedList[j].seed)[2] == 0)
    { 
      floodFill(colouredImage, seedList[j].seed, seedList[j].colour, &ccomp, Scalar(loDiff, loDiff, loDiff),
          Scalar(upDiff, upDiff, upDiff), flags);
    }
    //circle(colouredImage, seedList[j].seed, 5, (0,0,255), -1);
  }
  // for(int i=0;i<colouredImage.rows;i++) {
  //   for(int j=0;j<colouredImage.cols;j++) {
  //     if(colouredImage.at<Vec3b>(i,j)[0] == 0) colouredImage.at<Vec3b>(i,j) = Vec3b(255,0,255);
  //   }
  // }

  //resize(receivedImage, receivedImage, Size(), 2, 2, CV_INTER_CUBIC);
  //resize(receivedImage2, receivedImage2, Size(), 2, 2, CV_INTER_CUBIC);
  // for(int i=0;i<receivedImage2.rows;i++) {
  //   for(int j=0;j<receivedImage2.cols;j++) {
  //     if(receivedImage2.at<Vec3b>(i,j)[0] < 170) receivedImage2.at<Vec3b>(i,j) = Vec3b(0,0,0);
  //     else receivedImage2.at<Vec3b>(i,j) = Vec3b(255,255,255);
  //   }
  // }
  remap(colouredImage, colouredImage, lmapx, lmapy, cv::INTER_LINEAR);
  resize(colouredImage, colouredImage, Size(), 2, 2, CV_INTER_CUBIC);

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
  // consist_vis = filtered_disp_vis.clone();
  // if(first) {
  // 	last_vis = filtered_disp_vis.clone();
  // 	first = 0;
  // }
  // for(int i=0;i<filtered_disp_vis.rows;i++) {
  // 	uchar* filteredP = filtered_disp_vis.ptr<uchar>(i);
  // 	uchar* lastP = last_vis.ptr<uchar>(i);
  // 	//uchar* last2P = last2_vis.ptr<uchar>(i);
  // 	uchar* consistP = consist_vis.ptr<uchar>(i);
  //   for(int j=0;j<filtered_disp_vis.cols;j++) {
  //     if(abs(filteredP[j]-lastP[j]) > 100) consistP[j] = lastP[j];
  //     //cout << (int)filteredP[j] << " ";
  //   }
  // }
  // last_vis = filtered_disp_vis.clone();

  accumulateWeighted(filtered_disp_vis, weighted_map, 0.4);
  convertScaleAbs(weighted_map, output_map);

  //resize(receivedImage, receivedImage, Size(), 2, 2, CV_INTER_CUBIC);
  resize(receivedImage2, receivedImage2, Size(), 2, 2, CV_INTER_CUBIC);
  resize(output_map, output_map, Size(), 2, 2, CV_INTER_CUBIC);

  seedList.clear();
  imageBuf.clear();
  image2Buf.clear();
  imshow( "Edge Map", colouredImage ); 
  imshow( "Edge Map 2", receivedImage2 ); 
  imshow( "Depth", output_map );
}

void UDPReceive()
{
  this_thread::sleep_for(chrono::milliseconds(500));
  recvlen = recvfrom(fd, recvBuf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
  printf("received %d bytes\n", recvlen);
  if (recvlen > 0) { 
    showm.lock();
    msglenbuf.push_back(recvlen);
    for(int i=0;i<recvlen;i++) messageBuf[i] = recvBuf[i];
    showm.unlock();
  }
  addressReceived = 1;
  while(!finished)
  {
    recvlen = recvfrom(fd, recvBuf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
    printf("received %d bytes\n", recvlen);
    if (recvlen > 0) {
      showm.lock();
      msglenbuf.push_back(recvlen);
      for(int i=0;i<recvlen;i++) messageBuf[i] = recvBuf[i];
      showm.unlock();
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

  int num_frames = 500;
  time_t start, end;

  t = thread(UDPReceive);
  control = thread(SendControl);

  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map", 0, 40 ); 
  namedWindow("Edge Map 2", 1);
  cvMoveWindow( "Edge Map 2",  500, 40 );
  namedWindow("Depth", 1);
  cvMoveWindow( "Depth",  40, 500 );
  cvMoveWindow( "Edge Map 2", 2*320 + 70, 40 );
  cvMoveWindow( "Depth", 0, 2*240 + 70 );
  receivedImage.create(240,320,CV_8UC3);
  receivedImage2.create(240,320,CV_8UC3);
  weighted_map.create(240,320, CV_64FC1);
  initUndistortRectifyMap(K1, D1, R1, P1, receivedImage.size(), CV_32F, lmapx, lmapy);
  initUndistortRectifyMap(K2, D2, R2, P2, receivedImage2.size(), CV_32F, rmapx, rmapy);

  Mat leftDisplay, rightDisplay, disp;


  time(&start);
  /// Wait until user exit program by pressing a key
  int k;
  while(frames != num_frames) {
    if(msglenbuf.size() > 0) {
      showm.lock();
      msglen = msglenbuf.back();
      msglenbuf.clear();
      for(int i=0;i<msglen;i++) buf[i] = messageBuf[i];
      showm.unlock();
      ReceivePoints();
      frames++;
    }
  	/// Wait until user exit program by pressing a key
  	k = waitKey(1);
  }
  time(&end);
  double seconds = difftime(end, start);
  cout << "Time taken: " << seconds << " seconds" << endl;
  double fps = num_frames/seconds;
  cout << "FPS: " << fps << endl;
  finished = 1;
  shutdown(fd, SHUT_RDWR);
  close(fd);
  t.join();
  control.join();
  //imwrite("abstest.jpg", output_map);
  return 0;
}