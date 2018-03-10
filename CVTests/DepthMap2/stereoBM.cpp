#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

int main(int argc, char** argv)
{
    int wsize = 15;
    int max_disp = 112;
    double lambda = 8000;
    double sigma = 1.5;
    double vis_mult = 1;

    //! [load_views]
    Mat left  = imread(argv[1], IMREAD_COLOR);
    if ( left.empty() )
    {
        cout<<"Cannot read image file: "<<argv[1];
        return -1;
    }

    Mat right = imread(argv[2], IMREAD_COLOR);
    if ( right.empty() )
    {
        cout<<"Cannot read image file: "<<argv[2];
        return -1;
    }
    //! [load_views]
    Mat left_for_matcher, right_for_matcher;
    Mat map_x, map_y;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Mat conf_map = Mat(left.rows,left.cols,CV_8U);
    conf_map = Scalar(255);
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;

    left_for_matcher.create(left.rows+4,left.cols+4,left.type());
    right_for_matcher.create(right.rows+4,right.cols+4,right.type());
    left_for_matcher = Scalar(255,255,255,255);
    right_for_matcher = Scalar(255,255,255,255);
    left.copyTo(left_for_matcher(Rect(2,2,left.cols,left.rows)));
    right.copyTo(right_for_matcher(Rect(2,2,right.cols,right.rows)));

    //! [matching]
    Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
    wls_filter = createDisparityWLSFilter(left_matcher);
    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

    cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
    cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

    matching_time = (double)getTickCount();
    left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
    right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
    //! [matching]

    //! [filtering]
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp,left_for_matcher,filtered_disp,right_disp);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    //! [filtering]
    conf_map = wls_filter->getConfidenceMap();

    //collect and print all the stats:
    cout.precision(2);
    cout<<"Matching time:  "<<matching_time<<"s"<<endl;
    cout<<"Filtering time: "<<filtering_time<<"s"<<endl;
    cout<<endl;

    Mat filtered_disp_vis;
    getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    
    namedWindow("left", WINDOW_AUTOSIZE);
    imshow("left", left_for_matcher);
    namedWindow("right", WINDOW_AUTOSIZE);
    imshow("right", right_for_matcher);

    //! [visualization]
    Mat raw_disp_vis, out;
    getDisparityVis(left_disp,raw_disp_vis,vis_mult);
    raw_disp_vis.copyTo( out, left_for_matcher);
    namedWindow("raw disparity", WINDOW_AUTOSIZE);
    imshow("raw disparity", out);
    imwrite("result.jpg",out);

    namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    imshow("filtered disparity", filtered_disp_vis);
    waitKey();
    //! [visualization]

    return 0;
}