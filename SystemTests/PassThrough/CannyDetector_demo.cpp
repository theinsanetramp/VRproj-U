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
#include <wiringSerial.h>

#include "sensor.h"

#define GPIO_DIR_L 8
#define GPIO_PWM_L 13
#define GPIO_DIR_R 23
#define GPIO_PWM_R 18 
#define GPIO_DIR_X 27
#define GPIO_DIR_Y 17
#define DUTY_CYCLE 500000

#define X_K 38.0 
#define X_I 0.00
#define X_D 15.0
#define X_DEADZONE 300
#define X_AXIS_CLOCK_LIM 28000
#define X_AXIS_ANTI_LIM 28000
#define MAX_X  50000 
#define Y_K 18
#define Y_I 0.0000
#define Y_D 23
#define Y_DEADZONE 500 
#define Y_AXIS_TOP_LIM 18000
#define Y_AXIS_BOT_LIM 18000
#define MAX_Y 5000

using namespace cv;
using namespace std;

#define SERVICE_PORT  21234
#define BUFLEN 409600
#define RECEIVEBUFLEN 16

#define FRAMERATE 30
#define POINTSPERTHREAD 20

Mat tmp_frame, reduced_frame;
Mat tmp_frame2, reduced_frame2;

thread control;
int finished = 0;
int driving = 0;
mutex controlm;
condition_variable controlVar;

VideoCapture cap;
VideoCapture cap2;

struct sockaddr_in myaddr, remaddr;
int fd, bufSize, k, recvlen;
socklen_t slen=sizeof(remaddr);
//char server[] = "10.42.0.1";
uchar buf[BUFLEN];
vector<unsigned char> dstBuf;
vector<unsigned char> dst2Buf;
uchar receiveBuf[RECEIVEBUFLEN];

int nextLowThreshold = 30;
int serial;

vector<int> compressionParams;

Mat R1, R2, P1, P2, Q;
Mat K1, K2, R;
Vec3d T;
Mat D1, D2;
Mat lmapx, lmapy, rmapx, rmapy;
Mat imgU1, imgU2;

void ReceiveUDP()
{
  signed char XSign, YSign, GimbleX, GimbleY;
  int RSpeed = 0;
  int LSpeed = 0;
  int mode = 1;
  serial = serialOpen("/dev/ttyAMA0", 115200);
  if(serial == -1) {
      printf("Couldn't open serial port, exiting.\n");
      exit(-1);
  }
  //Sensor sensor;
  //SensorData gyro;
  
  double x_integral = 0;
  double y_integral = 0;
  double x_derivative = 0;
  double y_derivative = 0;
  double x_axis_sum = 0;
  double y_axis_sum = 0;
  
  while(!finished)
  {
    {
      unique_lock<mutex> lk(controlm);
      while(!driving) {
		  controlVar.wait(lk);
	  }
      //cout << "running" << endl;
      lk.unlock();
    }
    recvlen = recvfrom(fd, receiveBuf, RECEIVEBUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
    if(recvlen > 0) {
      XSign = receiveBuf[0];
      YSign = receiveBuf[1];
      GimbleX = receiveBuf[3];
      GimbleY = receiveBuf[4];
      if((int)receiveBuf[5] != mode-1) {
		cout << "Drive mode = " << mode << endl;
	    mode = receiveBuf[5]+1;
	  }
      if((int)XSign > 80) {
		  RSpeed = -90;
		  LSpeed = 90;
	  }
	  else if((int)XSign < -80) {
		  RSpeed = 90;
		  LSpeed = -90;
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
      gpioHardwarePWM(GPIO_PWM_R, 100, (mode*DUTY_CYCLE*abs(RSpeed))/127);
      gpioHardwarePWM(GPIO_PWM_L, 100, (mode*DUTY_CYCLE*abs(LSpeed))/127);
      
      if((int)receiveBuf[2] != nextLowThreshold) {
        nextLowThreshold = receiveBuf[2];
        cout << "Next low threshold: " << nextLowThreshold << endl;
      }
      int x_control = 8*((int)GimbleX);
      int y_control = 8*((int)GimbleY);
      /*gyro = sensor.getGyroData();
      double gyro_X = ((abs(gyro.x) < GYRO_X_THRESHOLD) ? 0 : gyro.x * GYRO_X_GAIN);
	  double gyro_Y = ((abs(gyro.y) < GYRO_Y_THRESHOLD) ? 0 : gyro.y * GYRO_Y_GAIN);
	  if(gyro_X > 0) printf("Using X Gyro\n");
	  double x_P = errorhorizontal * X_K;
	  x_integral += errorhorizontal;
	  double x_I = x_integral * X_I;
	  double x_D = X_D * (x_derivative - errorhorizontal);
	  x_derivative = errorhorizontal;
	  double y_P = errorvertical * Y_K;
	  y_integral += errorvertical;
	  double y_I = y_integral * Y_I;
	  double y_D = Y_D * (y_derivative - errorvertical);
	  y_derivative = errorvertical;
	  double x_control = x_P + x_I - x_D + gyro_X;
	  double y_control = y_P + y_I - y_D + gyro_Y;*/
	  int x_dir = (x_control < 0) ? 0 : 1;
	  int y_dir = (y_control < 0) ? 0 : 1;
      gpioWrite(GPIO_DIR_X, x_dir);
      gpioWrite(GPIO_DIR_Y, y_dir);
	  
	  /*if(fabs(x_control) < X_DEADZONE) {
		x_control = 0;
	  }
	  if(fabs(y_control) < Y_DEADZONE) {
		y_control = 0;
	  }*/


	  if(y_axis_sum > Y_AXIS_TOP_LIM && y_control > 0) {
	  	y_control = 0;
		printf("Hit y axis top limit\n");
	  } else if (y_axis_sum < -Y_AXIS_BOT_LIM && y_control < 0) {
		y_control = 0;
		printf("Hit y axis bottom limit\n");
	  }
	  y_axis_sum += y_control;
	  //cout << y_axis_sum << endl;

	  if(x_axis_sum > X_AXIS_ANTI_LIM && x_control > 0) {
		x_control = 0;
		printf("Hit x axis rotation limit\n");
  	  } else if (x_axis_sum < -X_AXIS_CLOCK_LIM && x_control < 0) {
		x_control = 0;
		printf("Hit x axis rotation limit\n");
	  }
	  x_axis_sum += x_control;
		
	  //Convert control input to pwm input
	  int x_axis_freq = abs((int)x_control);
	  int y_axis_freq = abs((int)y_control);
		
	  if(x_axis_freq > MAX_X) {
		x_axis_freq = 0;
		printf("!!! X axis error too high!!!\n");
	  }
	  if(y_axis_freq > MAX_Y) {
	  	y_axis_freq = 0;
		printf("!!! Y axis error too high!!!\n");
	  }

	  serialPrintf(serial, "%06d%06d", x_axis_freq, y_axis_freq);
    }
  }
}

void MainProcess(int, void*)
{
  
  cap.grab();
  cap2.grab();
  cap.retrieve(tmp_frame);
  cap2.retrieve(tmp_frame2);
  /// Convert the image to grayscale
  resize(tmp_frame, reduced_frame, Size(), 0.5, 0.5, CV_INTER_AREA);
  resize(tmp_frame2, reduced_frame2, Size(), 0.5, 0.5, CV_INTER_AREA);
  //remap(reduced_frame, reduced_frame, lmapx, lmapy, cv::INTER_LINEAR);
  //remap(reduced_frame2, reduced_frame2, rmapx, rmapy, cv::INTER_LINEAR);

  imshow( "Camera2", reduced_frame2 );
  imshow( "Camera", reduced_frame );

  imencode(".jpg", reduced_frame, dstBuf, compressionParams);
  imencode(".jpg", reduced_frame2, dst2Buf, compressionParams);

  for(int i=0;i<dstBuf.size();i++) buf[i] = dstBuf[i];
  for(int i=0;i<14;i++) buf[dstBuf.size() + i] = 0;
  for(int i=0;i<dst2Buf.size();i++) buf[14 + dstBuf.size() + i] = dst2Buf[i];
  bufSize = 14 + dstBuf.size() + dst2Buf.size();
  //cout << bufSize << endl;
  if (sendto(fd, buf, bufSize, 0, (struct sockaddr *)&remaddr, slen)==-1) {
      perror("sendto");
      {
        lock_guard<mutex> lk(controlm);
        driving = 0;
        cout << "control stopped\n";
      }
      gpioHardwarePWM(GPIO_PWM_R, 100, 0);
      gpioHardwarePWM(GPIO_PWM_L, 100, 0);
      serialPrintf(serial, "%06d%06d", 0, 0);
    }
    else if(!driving) {
	  {
        lock_guard<mutex> lk(controlm);
        driving = 1;
        cout << "control restarted\n";
      }
      controlVar.notify_one();
	}
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
  
  if (gpioInitialise() < 0) {
	cout << "gpioInitialise() failed\n";
    exit(1);
  }
  
  compressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
  compressionParams.push_back(70);

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
  /*resize(tmp_frame, reduced_frame, Size(), 0.5, 0.5, CV_INTER_AREA);
  
  cv::FileStorage fs1("cam_stereo.yml", cv::FileStorage::READ);
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
  initUndistortRectifyMap(K1, D1, R1, P1, reduced_frame.size(), CV_32F, lmapx, lmapy);
  initUndistortRectifyMap(K2, D2, R2, P2, reduced_frame.size(), CV_32F, rmapx, rmapy);*/

  //namedWindow( "Edge Map", 1 );
  //cvMoveWindow( "Edge Map", 0, 40 );
  namedWindow("Camera", 1);
  namedWindow("Camera2", 1);
  cvMoveWindow( "Camera2", tmp_frame.cols/2 + 20, 40 );

  control = thread(ReceiveUDP);
  cout << "starting\n";
  for(;;)
  {
    MainProcess(0, 0);

    char keycode = (char)waitKey(1);
      if( keycode == 27 ){
          //imwrite("frame.jpg", dst);
          break;
        }
  }
  finished = 1;
  shutdown(fd, SHUT_RDWR);
  close(fd);
  control.join();
  gpioTerminate();
  cout << "control thread joined\n";
  return 0;
  }
