#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "sobol.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "libimagequant.h"
#include "lodepng.h"
#include "lodepng.c"

using namespace cv;
using namespace std;

Mat src, src_reduced, dst_out;
Mat dst, flood_mask;
Mat receivedImage;

struct SeedData
{
  Point seed;
  Scalar colour;
};

vector<SeedData> seedList;

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

SeedData MakeSeedData(Point seed, Scalar colour)
{
  SeedData s;
  s.seed = seed;
  s.colour = colour;
  return s;
}

void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 5x5
  blur( dst, dst, Size(5,5) );
  //imwrite("buildBlur.jpg",detected_edges);
  /// Canny detector
  Canny( dst, dst, lowThreshold, lowThreshold*ratio, kernel_size );
  //imwrite("buildCanny.jpg",detected_edges);
  /// Apply the dilation operation
  dilate( dst, dst, element );
  //resize(dst, dst, Size(), 0.5, 0.5, CV_INTER_AREA);
  dst_out = dst.clone();
  //imwrite("buildDilated.jpg",dst);
  cvtColor(dst, dst, CV_GRAY2RGB); 

  //Floodfill from quasi-random points
  for (unsigned long long i = 0; i < 600; ++i)
  {
    int x = dst.cols * sobol::sample(i, 0);
    int y = dst.rows * sobol::sample(i, 1);
    Point seed = Point(x, y);
    if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
    {
      flood_mask = 0;
      floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
              Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8));
      Scalar newVal = mean(src_reduced,flood_mask);
      seedList.push_back(MakeSeedData(seed, newVal));
      floodFill(dst, seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
              Scalar(upDiff, upDiff, upDiff), flags);
    }
  }
  
  uchar* raw_rgba_pixels = new uchar[dst_out.total()*4];
  Mat continuousRGBA(dst_out.size(), CV_8UC4, raw_rgba_pixels);
  cvtColor(dst_out, continuousRGBA, CV_GRAY2RGBA, 4);

  // Use libimagequant to make a palette for the RGBA pixels
  liq_attr *handle = liq_attr_create();
  liq_image *input_image = liq_image_create_rgba(handle, raw_rgba_pixels, dst_out.cols, dst_out.rows, 0);
  // You could set more options here, like liq_set_quality
  liq_result *quantization_result;
  if (liq_image_quantize(input_image, handle, &quantization_result) != LIQ_OK) {
      fprintf(stderr, "Quantization failed\n");
  }

  // Use libimagequant to make new image pixels from the palette
  size_t pixels_size = dst_out.cols * dst_out.rows;
  unsigned char *raw_8bit_pixels = (unsigned char*)malloc(pixels_size);
  liq_set_dithering_level(quantization_result, 1.0);

  liq_write_remapped_image(quantization_result, input_image, raw_8bit_pixels, pixels_size);
  const liq_palette *palette = liq_get_palette(quantization_result);

  LodePNGState state;
  lodepng_state_init(&state);
  state.info_raw.colortype = LCT_PALETTE;
  state.info_raw.bitdepth = 8;
  state.info_png.color.colortype = LCT_PALETTE;
  state.info_png.color.bitdepth = 8;

  for(int i=0; i < palette->count; i++) {
     lodepng_palette_add(&state.info_png.color, palette->entries[i].r, palette->entries[i].g, palette->entries[i].b, palette->entries[i].a);
     lodepng_palette_add(&state.info_raw, palette->entries[i].r, palette->entries[i].g, palette->entries[i].b, palette->entries[i].a);
  }

  unsigned char *output_file_data;
  size_t output_file_size;
  unsigned int out_status = lodepng_encode(&output_file_data, &output_file_size, raw_8bit_pixels, dst_out.cols, dst_out.rows, &state);
  if (out_status) {
      fprintf(stderr, "Can't encode image: %s\n", lodepng_error_text(out_status));
  }
  cout << output_file_size << endl;

  // Done. Free memory.
  liq_result_destroy(quantization_result); // Must be freed only after you're done using the palette
  liq_image_destroy(input_image);
  liq_attr_destroy(handle);

  free(raw_8bit_pixels);
  lodepng_state_cleanup(&state);

  vector<unsigned char> imageBuf(output_file_data, output_file_data + output_file_size);

  receivedImage = imdecode(imageBuf, IMREAD_COLOR);
  resize(receivedImage, receivedImage, Size(), 2, 2, CV_INTER_CUBIC);
  //erode(receivedImage, receivedImage, element);
  for(int i=0;i<receivedImage.rows;i++) {
    for(int j=0;j<receivedImage.cols;j++) {
      if(receivedImage.at<Vec3b>(i,j)[0] < 170) receivedImage.at<Vec3b>(i,j) = Vec3b(0,0,0);
      else receivedImage.at<Vec3b>(i,j) = Vec3b(255,255,255);
    }
  }
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
    //else circle(receivedImage, seedList[j].seed, 5, 255, -1);
  }
  imshow( "Edge Map", dst );
  imshow( "Received Image", receivedImage );
 }


/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  flood_mask.create(src.rows/2+2, src.cols/2+2, CV_8UC1);

  /// Convert the image to grayscale
  resize(src, src_reduced, Size(), 0.5, 0.5, CV_INTER_AREA);
  cvtColor( src_reduced, dst, CV_BGR2GRAY );
  //imwrite("buildGray.jpg",src_gray);
  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map", 0, 40 );
  namedWindow( "Received Image", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Received Image", 1.1*src.cols, 40 );

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  int k;
  do{
  	 /// Wait until user exit program by pressing a key
  	 k = waitKey(0);
  }
  while(k != 27);
  imwrite("Final.png",dst_out);
  return 0;
  }