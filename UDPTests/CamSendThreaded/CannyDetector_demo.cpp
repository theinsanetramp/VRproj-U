#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video/background_segm.hpp"
#include "sobol.h"
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
#include "libimagequant.h"
#include "lodepng.h"
#include "lodepng.c"

using namespace cv;
using namespace std;

#define SERVICE_PORT  21234
#define BUFLEN 40960

#define FRAMERATE 60
#define POINTSPERTHREAD 10

Mat tmp_frame, reduced_frame, dst_out;
Mat tmp_frame2, reduced_frame2, dst_out2;
Mat dst, dst2, flood_mask[4], mean_mask[4];
//Mat sub_dst[4];

thread t[4];
thread capturet;
thread process2t;
int finished = 0;
int threadProcessing[4] = {0,0,0,0};
int capReady = 1;
int proc2Ready = 1;
mutex waitm;
mutex capwaitm;
mutex proc2waitm;
condition_variable conVar;
condition_variable conVar1;
condition_variable capVar;
condition_variable proc2Var;
mutex bufm;

VideoCapture cap;
VideoCapture cap2;
//Canny variables
int lowThreshold = 30;
int const max_lowThreshold = 100;
int thresh_ratio = 3.5;
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
//blur variables
int blur_kernel = 5;
int max_blur_kernel = 10;
//Final image saturation
int alpha = 1.2;

struct sockaddr_in myaddr, remaddr;
int fd, bufSize, k, slen=sizeof(remaddr);
//char server[] = "10.42.0.1";
uchar buf[BUFLEN];
int bufLength;
vector<unsigned char> dstBuf;

void FindColours(int corner, int sobolPoints)
{
  //Floodfill from quasi-random points
  for (unsigned long long i = 100 + sobolPoints*corner; i < (100 + sobolPoints*(corner+1)); i++)
  {
    int x = dst.cols * sobol::sample(i, 0);
    int y = dst.rows * sobol::sample(i, 1);
    Point seed = Point(x, y);
    if(dst.at<Vec3b>(seed)[0] == 0)
    {
      flood_mask[corner] = 0;
      floodFill(dst, flood_mask[corner], seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
              Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8));
      flood_mask[corner](Rect(1,1,flood_mask[corner].cols-1,flood_mask[corner].rows-1)).copyTo(mean_mask[corner]);
      Scalar newVal = alpha*mean(reduced_frame,mean_mask[corner]);
      if(newVal == Scalar(0,0,0)) newVal = Scalar(1,1,1);
      //cout << "processed" << corner << endl;
      bufm.lock();
      buf[bufLength*7] = (x >> 8) & 0xFF;
      buf[bufLength*7+1] = x & 0xFF;
      buf[bufLength*7+2] = (y >> 8) & 0xFF;
      buf[bufLength*7+3] = y & 0xFF;
      buf[bufLength*7+4] = newVal[0];
      buf[bufLength*7+5] = newVal[1];
      buf[bufLength*7+6] = newVal[2];
      bufLength++;
      bufm.unlock();
    }
  }
}

void FindColoursThread(int corner, int sobolPoints)
{
  while(!finished)
  {
    {
      unique_lock<mutex> lk(waitm);
      while(threadProcessing[corner] != 1) conVar.wait(lk);
      //cout << "running" << corner << endl;
      lk.unlock();
    }
    if(finished) break;
    FindColours(corner, sobolPoints);
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
  while(!finished)
  {
    {
      unique_lock<mutex> lk(capwaitm);
      while(capReady) capVar.wait(lk);
      //cout << "notified cap\n";
      capReady = 1;
      lk.unlock();
    }
    cap.grab();
    cap2.grab();
    cap.retrieve(tmp_frame);
    cap2.retrieve(tmp_frame2);
    if( tmp_frame.empty() || tmp_frame2.empty())
        break;
  }
}

vector<unsigned char> CompressImage(Mat image)
{
  liq_attr *handle = liq_attr_create();
  liq_image *input_image;
  liq_result *quantization_result;
  size_t pixels_size;
  unsigned char *raw_8bit_pixels;
  const liq_palette *palette;
  LodePNGState state;
  unsigned char *output_file_data;
  size_t output_file_size;
  liq_set_dithering_level(quantization_result, 1.0);

  uchar* raw_rgba_pixels = new uchar[dst_out.total()*4];
  Mat continuousRGBA(dst_out.size(), CV_8UC4, raw_rgba_pixels);
  cvtColor(dst_out, continuousRGBA, CV_GRAY2RGBA, 4);

  input_image = liq_image_create_rgba(handle, raw_rgba_pixels, dst_out.cols, dst_out.rows, 0);
  // You could set more options here, like liq_set_quality
  if (liq_image_quantize(input_image, handle, &quantization_result) != LIQ_OK) {
      fprintf(stderr, "Quantization failed\n");
  }

  pixels_size = dst_out.cols * dst_out.rows;
  raw_8bit_pixels = (unsigned char*)malloc(pixels_size);

  liq_write_remapped_image(quantization_result, input_image, raw_8bit_pixels, pixels_size);
  palette = liq_get_palette(quantization_result);

  lodepng_state_init(&state);
  state.info_raw.colortype = LCT_PALETTE;
  state.info_raw.bitdepth = 8;
  state.info_png.color.colortype = LCT_PALETTE;
  state.info_png.color.bitdepth = 8;

  for(int i=0; i < palette->count; i++) {
     lodepng_palette_add(&state.info_png.color, palette->entries[i].r, palette->entries[i].g, palette->entries[i].b, palette->entries[i].a);
     lodepng_palette_add(&state.info_raw, palette->entries[i].r, palette->entries[i].g, palette->entries[i].b, palette->entries[i].a);
  }

  unsigned int out_status = lodepng_encode(&output_file_data, &output_file_size, raw_8bit_pixels, dst_out.cols, dst_out.rows, &state);
  if (out_status) {
      fprintf(stderr, "Can't encode image: %s\n", lodepng_error_text(out_status));
  }

  liq_result_destroy(quantization_result); // Must be freed only after you're done using the palette
  liq_image_destroy(input_image);
  liq_attr_destroy(handle);

  free(raw_8bit_pixels);
  lodepng_state_cleanup(&state);

  vector<unsigned char> tmpBuf(output_file_data, output_file_data + output_file_size);
  return tmpBuf;
}

void Process2Thread()
{
  while(!finished)
  {
    {
      unique_lock<mutex> lk(proc2waitm);
      while(proc2Ready) proc2Var.wait(lk);
      //cout << "notified cap\n";
      proc2Ready = 1;
      lk.unlock();
    }

    resize(tmp_frame2, reduced_frame2, Size(), 0.5, 0.5, CV_INTER_AREA);
    cvtColor( reduced_frame2, dst2, CV_BGR2GRAY );
    // Reduce noise with kernel defined by blur_kernel
    blur( dst2, dst2, Size(blur_kernel,blur_kernel) );

    /// Canny detector
    Canny( dst2, dst2, lowThreshold, lowThreshold*thresh_ratio, kernel_size );

    /// Apply the dilation operation
    dilate( dst2, dst2, element );
    //cvtColor(dst, dst, CV_GRAY2RGB);
    dst_out2 = dst2.clone(); 
  }
}

void CannyThreshold(int, void*)
{
  /// Reduce noise with kernel defined by blur_kernel
  blur( dst, dst, Size(blur_kernel,blur_kernel) );

  /// Canny detector
  Canny( dst, dst, lowThreshold, lowThreshold*thresh_ratio, kernel_size );

  /// Apply the dilation operation
  dilate( dst, dst, element );
  //cvtColor(dst, dst, CV_GRAY2RGB);
  dst_out = dst.clone(); 
  {
    lock_guard<mutex> lk(waitm);
    for(int i=0;i<4;i++) threadProcessing[i] = 1;
    //cout << "notified main" << endl;
  }
  conVar.notify_all();
  //imencode(".png", dst_out, dstBuf);
  dstBuf = CompressImage(dst_out);
  //cout << output_file_size << endl;
  { 
    unique_lock<mutex> lk(waitm);
    while(threadProcessing[0] == 1 || threadProcessing[1] == 1 
      || threadProcessing[2] == 1 || threadProcessing[3] == 1) 
      conVar1.wait(lk);
    //cout << "main receieved " << endl;
    lk.unlock();
  }

  for(int i=0;i<7;i++) buf[bufLength*7 + i] = 0;
  for(int i=0;i<dstBuf.size();i++) buf[bufLength*7 + 7 + i] = dstBuf[i];
  bufSize = bufLength*7 + 7 + dstBuf.size();
  cout << bufLength << endl;
  bufLength = 0;
  imshow( "Edge Map", dst_out2 );
  imshow( "Camera", dst_out );
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
      if( !cap.isOpened() )
      {
        printf("Can not open camera 1\n");
        return -1;
      }
  }
  cap2.set(CV_CAP_PROP_FPS, FRAMERATE);
  cap.open(1);
  if( !cap.isOpened() )
  {
      printf("\nCan not open camera 1\n");
      cap.open(0);
      if( !cap.isOpened() )
      {
        printf("Can not open camera 0\n");
        return -1;
      }
  }
  cap.set(CV_CAP_PROP_FPS, FRAMERATE);
  int FPS = cap.get(CV_CAP_PROP_FPS);
  cout << "FPS Limit: " << FPS << endl;
  cout << "Low Threshold: " << lowThreshold << endl;
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

  namedWindow( "Edge Map", 1 );
  cvMoveWindow( "Edge Map", 0, 40 );
  namedWindow("Camera", 1);
  cvMoveWindow( "Camera", tmp_frame.cols + 20, 40 );
  namedWindow("Camera2", 1);
  cvMoveWindow( "Camera2", 20, tmp_frame.rows + 40 );
  //createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );
  //createTrackbar( "Blur Kernel Size:", "Edge Map", &blur_kernel, max_blur_kernel, 0 );

  capturet = thread(CaptureThread);
  process2t = thread(Process2Thread);
  for(int i=0;i<4;i++) t[i] = thread(FindColoursThread, i, POINTSPERTHREAD);

  for(;;)
  {
    /// Create a matrix of the same type and size as src (for dst)
    for(int i=0;i<4;i++) flood_mask[i].create(tmp_frame.rows/2+2, tmp_frame.cols/2+2, CV_8UC1);
    for(int i=0;i<4;i++) mean_mask[i].create(tmp_frame.rows/2, tmp_frame.cols/2, CV_8UC1);

    {
      lock_guard<mutex> lk(proc2waitm);
      proc2Ready = 0;
      //cout << "notified main\n";
    }
    proc2Var.notify_one();

    //imshow("Camera", tmp_frame);
    imshow("Camera2", tmp_frame2);
    /// Convert the image to grayscale
    resize(tmp_frame, reduced_frame, Size(), 0.5, 0.5, CV_INTER_AREA);
    {
      lock_guard<mutex> lk(capwaitm);
      capReady = 0;
      //cout << "notified main\n";
    }
    capVar.notify_one();
    cvtColor( reduced_frame, dst, CV_BGR2GRAY );

    /// Show the image
    CannyThreshold(0, 0);

    if (sendto(fd, buf, bufSize, 0, (struct sockaddr *)&remaddr, slen)==-1)
      perror("sendto");

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
  {
    lock_guard<mutex> lk(capwaitm);
    capReady = 0;
  }
  capVar.notify_one();
  capturet.join();
  {
    lock_guard<mutex> lk(proc2waitm);
    proc2Ready = 0;
  }
  proc2Var.notify_one();
  process2t.join();
  //t.join();
  return 0;
  }
