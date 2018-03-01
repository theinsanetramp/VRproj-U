#include "FloodFill.h"

FloodFill::FloodFill(int corner, int points, Mat tmp_frame)
{
	index = corner;
	sobolPoints = points;
	loDiff = 0;
	upDiff = 0;
	connectivity = 4;
	newMaskVal = 255;
	flags = connectivity + (newMaskVal << 8) +
                FLOODFILL_FIXED_RANGE;
  alpha = 1.2;
  flood_mask.create(tmp_frame.rows/2+2, tmp_frame.cols/2+2, CV_8UC1);
  mean_mask.create(tmp_frame.rows/2, tmp_frame.cols/2, CV_8UC1);
}

void FloodFill::FindColours(Mat dst, Mat reduced_frame)
{
  //Floodfill from quasi-random points
  for (unsigned long long i = 100 + sobolPoints*index; i < (100 + sobolPoints*(index+1)); i++)
  {
    int x = dst.cols * sobol::sample(i, 0);
    int y = dst.rows * sobol::sample(i, 1);
    Point seed = Point(x, y);
    if(dst.at<Vec3b>(seed)[0] == 0)
    {
      flood_mask = 0;
      floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
              Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8));
      flood_mask(Rect(1,1,flood_mask.cols-2,flood_mask.rows-2)).copyTo(mean_mask);
      Scalar newVal = alpha*mean(reduced_frame,mean_mask);
      if(newVal == Scalar(0,0,0)) newVal = Scalar(1,1,1);
      //cout << "processed" << index << endl;
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