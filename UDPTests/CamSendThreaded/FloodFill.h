#ifndef FLOODFILL_H
#define FLOODFILL_H
 
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <mutex>
#include "sobol.h"

using namespace cv;
using namespace std;

extern mutex bufm;
extern uchar buf[];
extern int bufLength;

class FloodFill
{
private:
    //floodFill variables
    int loDiff, upDiff;
    int connectivity;
    int newMaskVal;
    int flags;
    Rect ccomp;
    int alpha;
    int index;
    int sobolPoints;
    Mat flood_mask;
    Mat mean_mask;

public:
    FloodFill(int corner, int points, Mat tmp_frame); 
    void FindColours(Mat dst, Mat reduced_frame);
};
 
#endif 
