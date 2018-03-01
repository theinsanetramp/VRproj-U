#ifndef COMPRESSOR_H
#define COMPRESSOR_H
 
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "lodepng.h"
#include "libimagequant.h"
#include <iostream>


using namespace cv;
using namespace std;

class Compressor
{
private:
    liq_attr *handle;
    liq_image *input_image;
    liq_result *quantization_result;
    size_t pixels_size;
    unsigned char *raw_8bit_pixels;
    const liq_palette *palette;
    LodePNGState state;
    unsigned char *output_file_data;
    size_t output_file_size;
    uchar* raw_rgba_pixels;
    Mat continuousRGBA;

    //Canny variables
    int lowThreshold;
    int thresh_ratio;
    int kernel_size;
    //dilation variables
    int dilation_type;
    int dilation_size;
    Mat element;
    //blur variables
    int blur_kernel;

public:
    Compressor(); 
    void CannyThreshold(Mat image);
    vector<unsigned char> CompressImage(Mat image);
    ~Compressor();
    int GetLowThreshold();
    void SetLowThreshold(int newLowThreshold);
};
 
#endif 
