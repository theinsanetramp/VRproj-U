#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video/background_segm.hpp"
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
#include <unistd.h>
#include <wiringPi.h>
#include <pigpio.h>

#include "compressor.h"
#include "FloodFill.h"

#define GPIO_DIR_L 8
#define GPIO_PWM_L 13
#define GPIO_DIR_R 23
#define GPIO_PWM_R 18 
#define DUTY_CYCLE 500000

using namespace cv;
using namespace std;

#define SERVICE_PORT  21234
#define BUFLEN 40960
#define RECEIVEBUFLEN 16

#define FRAMERATE 30
#define POINTSPERTHREAD 10

Mat tmp_frame, reduced_frame, dst_out;
Mat tmp_frame2, reduced_frame2;
Mat dst, dst2;

thread t[4];
thread capturet;
thread process2t;
thread out;
thread control;
int finished = 0;
int threadProcessing[4] = {0,0,0,0};
mutex waitm;
mutex bufm;
mutex capwaitm;
mutex proc2waitm;
mutex outwaitm;
condition_variable conVar;
condition_variable conVar1;
condition_variable capVar;
condition_variable proc2Var;
condition_variable outVar;

vector<Mat> cap2mainBuf;
vector<Mat> cap2proc2Buf;
vector<vector<unsigned char> > proc2outBuf;
vector<vector<unsigned char> > main2outBuf;

VideoCapture cap;
VideoCapture cap2;

Compressor compressor1;

struct sockaddr_in myaddr, remaddr;
int fd, bufSize, k, recvlen;
socklen_t slen=sizeof(remaddr);
//char server[] = "10.42.0.1";
uchar buf[BUFLEN];
int bufLength;
vector<unsigned char> dstBuf;
uchar receiveBuf[RECEIVEBUFLEN];

int nextLowThreshold = 30;

void ReceiveUDP()
{
  signed char XSign, YSign;
  int RSpeed = 0;
  int LSpeed = 0;
  if (gpioInitialise() < 0) {
	cout << "gpioInitialise() failed\n";
    exit(1);
  }
  while(!finished)
  {
    recvlen = recvfrom(fd, receiveBuf, RECEIVEBUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
    if(recvlen > 0) {
      XSign = receiveBuf[0];
      YSign = receiveBuf[1];
      if((int)XSign > 80) {
		  RSpeed = 127;
		  LSpeed = -127;
	  }
	  else if((int)XSign < -80) {
		  RSpeed = -127;
		  LSpeed = 127;
	  }
	  else {
		  RSpeed = (int)YSign;
		  LSpeed = (int)YSign;
	  }
      //cout << LSpeed << " " << RSpeed << endl;
      if(RSpeed >= 0) gpioWrite(GPIO_DIR_R, 1);
      else gpioWrite(GPIO_DIR_R, 0);
      if(LSpeed >= 0) gpioWrite(GPIO_DIR_L, 1);
      else gpioWrite(GPIO_DIR_L, 0);
      gpioHardwarePWM(GPIO_PWM_R, 100, (DUTY_CYCLE*abs(RSpeed))/127);
      gpioHardwarePWM(GPIO_PWM_L, 100, (DUTY_CYCLE*abs(LSpeed))/127);
      
      if((int)receiveBuf[2] != nextLowThreshold) {
        nextLowThreshold = receiveBuf[2];
        cout << "Next low threshold: " << nextLowThreshold << endl;
      }
    }
  }
}

void FindColoursThread(int corner, int sobolPoints)
{
  FloodFill filler = FloodFill(corner, sobolPoints, tmp_frame);
  while(!finished)
  {
    {
      unique_lock<mutex> lk(waitm);
      while(threadProcessing[corner] != 1) conVar.wait(lk);
      //cout << "running" << corner << endl;
      lk.unlock();
    }
    if(finished) break;
    filler.FindColours(dst, reduced_frame);
    {
      lock_guard<mutex> lk(waitm);
      threadProcessing[corner] = 0;
      //cout << "notified" << corner << endl;
    }
    conVar1.notify_one();
  }
}

void CaptureThread()
{
  Mat capture, capture2;
  while(!finished)
  {
    {
      unique_lock<mutex> lk(capwaitm);
      while(cap2mainBuf.size() == 1 || cap2proc2Buf.size() == 1) capVar.wait(lk);
      //cout << "notified cap\n";
      lk.unlock();
    }
    cap.grab();
    cap2.grab();
    cap.retrieve(capture);
    cap2.retrieve(capture2);
    if( capture.empty() || capture2.empty())
        break;
    {
      lock_guard<mutex> lk(capwaitm);
      cap2mainBuf.push_back(capture);
      //cout << "notified main\n";
    }
    capVar.notify_one();
    {
      lock_guard<mutex> lk(proc2waitm);
      cap2proc2Buf.push_back(capture2);
      //cout << "notified main\n";
    }
    proc2Var.notify_one();
  }
}

void Process2Thread()
{
  Compressor compressor2;
  vector<unsigned char> tmpBuf;
  while(!finished)
  {
    {
      unique_lock<mutex> lk(proc2waitm);
      while(cap2proc2Buf.size() == 0 || proc2outBuf.size() > 0) proc2Var.wait(lk);
      //cout << "notified proc2\n";
      tmp_frame2 = cap2proc2Buf.back();
      cap2proc2Buf.pop_back();
      lk.unlock();
    }
    capVar.notify_one();

    resize(tmp_frame2, reduced_frame2, Size(), 0.5, 0.5, CV_INTER_AREA);
    cvtColor( reduced_frame2, dst2, CV_BGR2GRAY );
    
    compressor2.SetLowThreshold(nextLowThreshold);
    compressor2.CannyThreshold(dst2);
    //dst2Buf = compressor2.CompressImage(dst2);
    imencode(".png", dst2, tmpBuf);
    {
      lock_guard<mutex> lk(outwaitm);
      proc2outBuf.push_back(tmpBuf);
      //cout << "notified proc2\n";
    }
    outVar.notify_one();
  }
}

void OutThread()
{
  vector<unsigned char> dst_outBuf;
  vector<unsigned char> dst2Buf;
  while(!finished)
  {
    {
      unique_lock<mutex> lk(outwaitm);
      while(proc2outBuf.size() == 0 || main2outBuf.size() == 0) outVar.wait(lk);
      //cout << "notified out\n";
      dst2Buf = proc2outBuf.back();
      proc2outBuf.pop_back();
      dst_outBuf = main2outBuf.back();
      main2outBuf.pop_back();
      lk.unlock();
    }
    proc2Var.notify_one();
    capVar.notify_one();
    
    for(int i=0;i<7;i++) buf[bufLength*7 + i] = 0;
    for(int i=0;i<dstBuf.size();i++) buf[bufLength*7 + 7 + i] = dstBuf[i];
    for(int i=0;i<7;i++) buf[bufLength*7 + 7 + dstBuf.size() + i] = 0;
    for(int i=0;i<dst2Buf.size();i++) buf[bufLength*7 + 14 + dstBuf.size() + i] = dst2Buf[i];
    //for(int i=0;i<14;i++) cout << (int)buf[bufLength*7 + 7 + dstBuf.size() + i] << endl;
    bufSize = bufLength*7 + 14 + dstBuf.size() + dst2Buf.size();
    //cout << bufSize << endl;
    bufLength = 0;

    if (sendto(fd, buf, bufSize, 0, (struct sockaddr *)&remaddr, slen)==-1)
      perror("sendto");
    //cout << "sent\n";
  }
}

void MainProcess(int, void*)
{
  {
    unique_lock<mutex> lk(capwaitm);
    while(cap2mainBuf.size() == 0 || main2outBuf.size() > 0) capVar.wait(lk);
    //cout << "notified main\n";
    tmp_frame = cap2mainBuf.back();
    cap2mainBuf.pop_back();
    lk.unlock();
  }
  capVar.notify_one();

  /// Convert the image to grayscale
  resize(tmp_frame, reduced_frame, Size(), 0.5, 0.5, CV_INTER_AREA);
  cvtColor( reduced_frame, dst, CV_BGR2GRAY );

  compressor1.SetLowThreshold(nextLowThreshold);
  compressor1.CannyThreshold(dst);
  dst_out = dst.clone(); 
  //imshow( "Edge Map", dst_out );
  imshow( "Camera", tmp_frame );
  {
    lock_guard<mutex> lk(waitm);
    for(int i=0;i<4;i++) threadProcessing[i] = 1;
    //cout << "notified main" << endl;
  }
  conVar.notify_all();
  imencode(".png", dst_out, dstBuf);
  //dstBuf = compressor1.CompressImage(dst_out);
  //cout << output_file_size << endl;
  { 
    unique_lock<mutex> lk(waitm);
    while(threadProcessing[0] == 1 || threadProcessing[1] == 1 
      || threadProcessing[2] == 1 || threadProcessing[3] == 1) 
      conVar1.wait(lk);
    //cout << "main receieved " << endl;
    lk.unlock();
  }
  {
    lock_guard<mutex> lk(outwaitm);
    main2outBuf.push_back(dstBuf);
    //cout << "notified main\n";
  }
  outVar.notify_one();
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

  cap2.open(2);
  if( !cap2.isOpened() )
  {
      printf("\nCan not open camera 2\n");
      cap2.open(1);
      cap.open(0);
      if( !cap2.isOpened() || !cap.isOpened())
      {
        printf("Can not open cameras\n");
        return -1;
      }
  }
  else cap.open(1);
  cap2.set(CV_CAP_PROP_FPS, FRAMERATE);
  cap.set(CV_CAP_PROP_FPS, FRAMERATE);
  int FPS = cap.get(CV_CAP_PROP_FPS);
  cout << "FPS Limit: " << FPS << endl;
  cout << "Low Threshold: " << compressor1.GetLowThreshold() << endl;
  cout << "Points per thread: " << POINTSPERTHREAD << endl;
  cap >> tmp_frame;
  if(tmp_frame.empty())
  {
      printf("can not read data from the video source 1\n");
      return -1;
  }
  cap2 >> tmp_frame2;
  if(tmp_frame2.empty())
  {
      printf("can not read data from the video source 2\n");
      return -1;
  }

  //namedWindow( "Edge Map", 1 );
  //cvMoveWindow( "Edge Map", 0, 40 );
  namedWindow("Camera", 1);
  cvMoveWindow( "Camera", tmp_frame.cols + 20, 40 );
  //namedWindow("Camera2", 1);
  //cvMoveWindow( "Camera2", 20, tmp_frame.rows + 40 );

  capturet = thread(CaptureThread);
  process2t = thread(Process2Thread);
  out = thread(OutThread);
  control = thread(ReceiveUDP);
  for(int i=0;i<4;i++) t[i] = thread(FindColoursThread, i, POINTSPERTHREAD);

  for(;;)
  {
    MainProcess(0, 0);

    char keycode = (char)waitKey(30);
      if( keycode == 27 ){
          //imwrite("frame.jpg", dst);
          break;
        }
  }
  finished = 1;
  {
    lock_guard<mutex> lk(waitm);
    for(int i=0;i<4;i++) threadProcessing[i] = 1;
  }
  conVar.notify_all();
  for(int i=0;i<4;i++) t[i].join();
  cout << "Fill threads joined\n";
  {
    lock_guard<mutex> lk(capwaitm);
    cap2mainBuf.pop_back();
  }
  capVar.notify_one();
  capturet.join();
  cout << "Capture thread joined\n";
  {
    lock_guard<mutex> lk(proc2waitm);
    cap2proc2Buf.push_back(tmp_frame2);
    proc2outBuf.pop_back();
  }
  proc2Var.notify_one();
  process2t.join();
  cout << "Proc2 thread joined\n";
  {
    lock_guard<mutex> lk(outwaitm);
    vector<unsigned char> empt;
    main2outBuf.push_back(empt);
    proc2outBuf.push_back(empt);
  }
  outVar.notify_one();
  out.join();
  cout << "out thread joined\n";
  shutdown(fd, SHUT_RDWR);
  close(fd);
  control.join();
  gpioTerminate();
  cout << "control thread joined\n";
  return 0;
  }
