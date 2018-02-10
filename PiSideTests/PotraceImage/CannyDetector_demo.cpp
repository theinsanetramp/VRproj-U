#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "sobol.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "potracelib.h"
#include <errno.h>

using namespace cv;
using namespace std;

/* ---------------------------------------------------------------------- */
/* auxiliary bitmap functions */

/* macros for writing individual bitmap pixels */
#define BM_WORDSIZE ((int)sizeof(potrace_word))
#define BM_WORDBITS (8*BM_WORDSIZE)
#define BM_HIBIT (((potrace_word)1)<<(BM_WORDBITS-1))
#define bm_scanline(bm, y) ((bm)->map + (y)*(bm)->dy)
#define bm_index(bm, x, y) (&bm_scanline(bm, y)[(x)/BM_WORDBITS])
#define bm_mask(x) (BM_HIBIT >> ((x) & (BM_WORDBITS-1)))
#define bm_range(x, a) ((int)(x) >= 0 && (int)(x) < (a))
#define bm_safe(bm, x, y) (bm_range(x, (bm)->w) && bm_range(y, (bm)->h))
#define BM_USET(bm, x, y) (*bm_index(bm, x, y) |= bm_mask(x))
#define BM_UCLR(bm, x, y) (*bm_index(bm, x, y) &= ~bm_mask(x))
#define BM_UPUT(bm, x, y, b) ((b) ? BM_USET(bm, x, y) : BM_UCLR(bm, x, y))
#define BM_PUT(bm, x, y, b) (bm_safe(bm, x, y) ? BM_UPUT(bm, x, y, b) : 0)

/* return new un-initialized bitmap. NULL with errno on error */
static potrace_bitmap_t *bm_new(int w, int h) {
  potrace_bitmap_t *bm;
  int dy = (w + BM_WORDBITS - 1) / BM_WORDBITS;
  bm = (potrace_bitmap_t *) malloc(sizeof(potrace_bitmap_t));
  if (!bm) {
    return NULL;
  }
  bm->w = w;
  bm->h = h;
  bm->dy = dy;
  bm->map = (potrace_word *) calloc(h, dy * BM_WORDSIZE);
  if (!bm->map) {
    free(bm);
    return NULL;
  }
  return bm;
}

/* free a bitmap */
static void bm_free(potrace_bitmap_t *bm) {
  if (bm != NULL) {
    free(bm->map);
  }
  free(bm);
}

/* ---------------------------------------------------------------------- */

potrace_bitmap_t *bm;
potrace_param_t *param;
potrace_path_t *p;
potrace_state_t *st;
int n, *tag;
potrace_dpoint_t (*c)[3];

Mat src;
Mat dst, flood_mask;
Mat epsConversion;

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

Point ConvertPoint(double x, double y)
{
  int u = 6*(int)round(10*x)/10;
  int v = epsConversion.rows - 6*(int)round(10*y)/10;
  return Point(u, v);
}

void ConvertLineTo(Point& brush, Point end)
{
  line(epsConversion, brush, end, Scalar(255,255,255));
  brush = end;
}

int getPt( int n1 , int n2 , float perc )
{
    int diff = n2 - n1;

    return n1 + ( diff * perc );
}    

void ConvertCurveTo(Point& brush, Point control1, Point control2, Point end)
{
  float xa, ya, xb, yb, xc, yc, xm, ym, xn, yn;
  Point tmp1 = brush;
  Point tmp2;
  for( float i = 0 ; i < 1 ; i += 0.01 )
  {
    xa = getPt( brush.x , control1.x , i );
    ya = getPt( brush.y , control1.y , i );
    xb = getPt( control1.x , control2.x , i );
    yb = getPt( control1.y , control2.y , i );
    xc = getPt( control2.x , end.x , i );
    yc = getPt( control2.y , end.y , i );

    // The Blue Line
    xm = getPt( xa , xb , i );
    ym = getPt( ya , yb , i );
    xn = getPt( xb , xc , i );
    yn = getPt( yb , yc , i );

    // The Black Dot
    tmp2.x = getPt( xm , xn , i );
    tmp2.y = getPt( ym , yn , i );

    // // The Green Line
    // xa = getPt( brush.x , control.x , i );
    // ya = getPt( brush.y , control.y , i );
    // xb = getPt( control.x , end.x , i );
    // yb = getPt( control.y , end.y , i );

    // // The Black Dot
    // tmp2.x = getPt( xa , xb , i );
    // tmp2.y = getPt( ya , yb , i );
    line(epsConversion, tmp1, tmp2, Scalar(255,255,255));
    tmp1 = tmp2;
    //epsConversion.at<float>(tmp2.y,tmp2.x) = 255;
  }
  brush = end;
}

struct FillLine
{
  Point start;
  Point end;
};

FillLine CreateFillLine(Point start, Point end)
{
  FillLine f;
  f.start = start;
  f.end = end;
  return f;
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
  int i = dst.rows - 1;
  for (int y=0; y<dst.rows/3; y++) {
    for (int x=0; x<dst.cols/3; x++) {
      BM_PUT(bm, x, y, (dst.at<Vec3b>(i,x)[0] > 0) ? 0 : 1);
    }
    i = i - 3;
  }
  //
  //cout << bm->w << "  " << dst.cols << endl;
  /* set tracing parameters, starting from defaults */
  param = potrace_param_default();
  if (!param) {
    fprintf(stderr, "Error allocating parameters: %s\n", strerror(errno)); 
    return;
  }
  param->turdsize = 6;
  param->opttolerance = 0.8;
  //param->turnpolicy = POTRACE_TURNPOLICY_WHITE;
  //param->alphamax = 0.8;

  /* trace the bitmap */
  st = potrace_trace(param, bm);
  if (!st || st->status != POTRACE_STATUS_OK) {
    fprintf(stderr, "Error tracing bitmap: %s\n", strerror(errno));
    return;
  }
  bm_free(bm);

  /* output vector data, e.g. as a rudimentary EPS file */
  printf("%%!PS-Adobe-3.0 EPSF-3.0\n");
  printf("%%%%BoundingBox: 0 0 %d %d\n", dst.cols/3, dst.rows/3);
  printf("gsave\n");

  /* draw each curve */
  Point brush;
  vector<vector<Point> > contours;
  vector<vector<Point> > keyPoints;
  vector<FillLine> fillLines;
  int j = 0;
  p = st->plist;
  while (p != NULL) {
    n = p->curve.n;
    tag = p->curve.tag;
    c = p->curve.c;
    vector<Point> tmp;
    contours.push_back(tmp);
    keyPoints.push_back(tmp);
    printf("%d %d moveto\n", (int)round(c[n-1][2].x), (int)round(c[n-1][2].y));
    brush = ConvertPoint(c[n-1][2].x, c[n-1][2].y);
    contours[j].push_back(brush);
    keyPoints[j].push_back(brush);
    for (int i=0; i<n; i++) {
      switch (tag[i]) {
      case POTRACE_CORNER:
        printf("%d %d lineto\n", (int)round(c[i][1].x), (int)round(c[i][1].y));
        ConvertLineTo(brush, ConvertPoint(c[i][1].x, c[i][1].y));
        contours[j].push_back(ConvertPoint(c[i][1].x, c[i][1].y));
        keyPoints[j].push_back(ConvertPoint(c[i][1].x, c[i][1].y));
        printf("%d %d lineto\n", (int)round(c[i][2].x), (int)round(c[i][2].y));
        ConvertLineTo(brush, ConvertPoint(c[i][2].x, c[i][2].y));
        contours[j].push_back(ConvertPoint(c[i][2].x, c[i][2].y));
        keyPoints[j].push_back(ConvertPoint(c[i][2].x, c[i][2].y));
        break;
      case POTRACE_CURVETO:
        printf("%d %d %d %d %d %d curveto\n", 
               (int)round(c[i][0].x), (int)round(c[i][0].y),
               (int)round(c[i][1].x), (int)round(c[i][1].y),
               (int)round(c[i][2].x), (int)round(c[i][2].y));
        ConvertCurveTo(brush,ConvertPoint(c[i][0].x, c[i][0].y),ConvertPoint(c[i][1].x, c[i][1].y),ConvertPoint(c[i][2].x, c[i][2].y));
        contours[j].push_back(ConvertPoint(c[i][0].x, c[i][0].y));
        contours[j].push_back(ConvertPoint(c[i][1].x, c[i][1].y));
        contours[j].push_back(ConvertPoint(c[i][2].x, c[i][2].y));
        keyPoints[j].push_back(ConvertPoint(c[i][2].x, c[i][2].y));
        break;
      }
    }
    /* at the end of a group of a positive path and its negative
       children, fill. */
    if (p->next == NULL || p->next->sign == '+') {
      printf("0 setgray fill\n");
      Point avgPoint;
      int pass = 0;
      int k = 0;
      do {
        avgPoint = Point((keyPoints[j][k].x + keyPoints[j][k+2].x)/2, (keyPoints[j][k].y + keyPoints[j][k+2].y)/2);
        if(pointPolygonTest(contours[j], avgPoint, false) == 1) {
          pass = 1;
          //cout << "success\n";
        } 
        else if(!(k >= keyPoints.size())) {
          k++;
          //cout << "next point\n";
        }
        else {
          //cout << "can't fill shape" << endl;
          break;
        }
      }
      while(!pass);
      fillLines.push_back(CreateFillLine(keyPoints[j][k], keyPoints[j][k+2]));
      j++;
    }
    else {
      contours.pop_back();
      keyPoints.pop_back();
    }
    p = p->next;
  }
  printf("grestore\n");
  printf("%%EOF\n");
  potrace_state_free(st);
  potrace_param_free(param);
  dilate( epsConversion, epsConversion, element );
  //erode( epsConversion, epsConversion, element );
  //cout << fillPoints << endl;
  // for(int i=0;i<fillLines.size();i++)
  // {
  //   Point fillPoint;
  //   float j = 0;
  //   int pass = 0;
  //   do
  //   {
  //     fillPoint.x = getPt(fillLines[i].start.x, fillLines[i].end.x, j);
  //     fillPoint.y = getPt(fillLines[i].start.y, fillLines[i].end.y, j);
  //     j += 0.001;
  //     if(epsConversion.at<uchar>(fillPoint) == 0) pass = 1;
  //     if(j > 1) {
  //       cout << "fillLines failed\n";
  //       pass = 1;
  //       fillPoint.x = epsConversion.cols;
  //     }
  //   }
  //   while(!pass);
  //   if(fillPoint.x < epsConversion.cols && fillPoint.y < epsConversion.rows) {
  //     floodFill(epsConversion, fillPoint, 255, &ccomp, Scalar(loDiff, loDiff, loDiff),
  //                 Scalar(upDiff, upDiff, upDiff), flags);
  //   }
  //   else cout << "outside image\n"; 
  //   circle(epsConversion, fillPoint, 5, 150, -1);
  //   //cout << fillLines[i].start << "  " << fillLines[i].end << endl;
  // }
  //imwrite("buildDilated.jpg",dst);
  cvtColor(dst, dst, CV_GRAY2RGB); 

  //Floodfill from quasi-random points
  for (unsigned long long i = 0; i < 600; ++i)
    {
        int x = src.cols * sobol::sample(i, 0);
        int y = src.rows * sobol::sample(i, 1);
        Point seed = Point(x, y);
        if(dst.at<Vec3b>(seed)[0] == 0 && dst.at<Vec3b>(seed)[1] == 0 && dst.at<Vec3b>(seed)[2] == 0)
        {
          flood_mask = 0;
          floodFill(dst, flood_mask, seed, (255,255,255), &ccomp, Scalar(loDiff, loDiff, loDiff),
                  Scalar(upDiff, upDiff, upDiff), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
          Scalar newVal = mean(src,flood_mask);
          floodFill(dst, seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                  Scalar(upDiff, upDiff, upDiff), flags);
          seed = Point(2*seed.x,2*seed.y);
          if(epsConversion.at<Vec3b>(seed)[0] == 0 && epsConversion.at<Vec3b>(seed)[1] == 0 && epsConversion.at<Vec3b>(seed)[2] == 0)
          {
            floodFill(epsConversion, seed, newVal, &ccomp, Scalar(loDiff, loDiff, loDiff),
                    Scalar(upDiff, upDiff, upDiff), flags);
          }
          //else circle(epsConversion, seed, 5, 255, -1);
        }
    }

  imshow( "Edge Map", dst );
  imshow( "Eps Conversion", epsConversion);
 }


/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );
  flood_mask.create(src.rows+2, src.cols+2, CV_8UC1);
  epsConversion.create( 2*src.rows, 2*src.cols, src.type() );
  // for(int i=0;i<epsConversion.rows;i++) {
  //   for(int j=0;j<epsConversion.cols;j++) {
  //     epsConversion.at<Vec3b>(i,j) = 200;
  //   }
  // }

  bm = bm_new(dst.cols/3, dst.rows/3);
  if (!bm) {
    fprintf(stderr, "Error allocating bitmap: %s\n", strerror(errno)); 
    return 1;
  }

  /// Convert the image to grayscale
  cvtColor( src, dst, CV_BGR2GRAY );
  //imwrite("buildGray.jpg",src_gray);
  namedWindow( "Edge Map", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Edge Map", 0, 40 );
  namedWindow( "Eps Conversion", CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Eps Conversion", 1.1*src.cols, 40 );

  /// Create a Trackbar for user to enter threshold
  //createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  int k;
  do{
  	 /// Wait until user exit program by pressing a key
  	 k = waitKey(0);
  }
  while(k != 27);
  imwrite("dst.jpg",dst);
  imwrite("redundancy.jpg", epsConversion);
  return 0;
  }