#include <iostream>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <timer/timer.h>

using namespace std;

int main(int argc, char** argv)
{
  cv::VideoCapture cap(argv[1]);
  if(!cap.isOpened()) {
    cout << "Failed to open " << argv[1] << endl;
    return 1;
  }

  cv::namedWindow("flow", 1);

  cv::Mat1b curr;
  cv::Mat1b prev;
  vector<cv::Point2f> curr_pts;
  vector<cv::Point2f> prev_pts;
  cv::Mat3b orig;
  cv::Mat3b curr3b;
  cv::Mat3b prev3b;
  cv::Mat3b flow_vis;
  cv::Mat3b features_vis;
  cv::Rect roi(390, 219, 1074, 343);
  cv::Mat1b delta;
  cv::Mat3b surf_vis;
  
  for(int i = 0; i < 100; ++i) cap >> orig;

  cap >> orig;
  cv::Mat(orig, roi).copyTo(curr3b);
  cv::cvtColor(curr3b, curr, CV_BGR2GRAY);

  int ms = 0;
  int num = 0;
  while(true) {
    ++num;
    prev = curr.clone();
    prev3b = curr3b.clone();
    
    cap >> orig;
    cv::Mat(orig, roi).copyTo(curr3b);
    cv::cvtColor(curr3b, curr, CV_BGR2GRAY);
        
    // -- goodFeaturesToTrack.
    prev_pts.clear();
    cv::goodFeaturesToTrack(prev, prev_pts, 500, 0.001, 10, cv::Mat(), 7);
    cv::Size subPixWinSize(10,10), winSize(21,21);
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
    
    features_vis = prev3b.clone();
    for(size_t i = 0; i < prev_pts.size(); ++i) {
      cv::circle(features_vis, prev_pts[i], 2, cv::Scalar(0, 0, 255), -1);
    }
    cv::imshow("features", features_vis);

    // -- Flow.
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(prev, curr, prev_pts, curr_pts,
                             status, err, winSize,
                             3, termcrit, 0, 0.001);

    flow_vis = prev3b.clone();
    for(size_t i = 0; i < status.size(); ++i) {
      if(status[i]) {
        cv::circle(flow_vis, prev_pts[i], 2, cv::Scalar(0, 0, 255), -1);
        cv::line(flow_vis, prev_pts[i], curr_pts[i], cv::Scalar(0, 0, 255), 1);
      }
    }
    cv::imshow("flow", flow_vis);
    
    // -- Delta image.
    delta = cv::Mat1b(curr.size(), 0);
    for(int y = 0; y < curr.rows; ++y)
      for(int x = 0; x < curr.cols; ++x)
        delta(y, x) = fabs(curr(y, x) - prev(y, x));
    cv::imshow("delta", delta);
    
    // -- SURF.
    surf_vis = prev3b.clone();
    cv::SURF surf(1000);
    vector<cv::KeyPoint> keypoints;
    //HighResTimer hrt("surf"); hrt.start();
    surf(prev3b, cv::Mat(), keypoints);
    //hrt.stop(); cout << hrt.reportMilliseconds() << endl;
    for(size_t i = 0; i < keypoints.size(); ++i)
      cv::circle(surf_vis, keypoints[i].pt, 2, cv::Scalar(0, 255, 0), -1);

    ostringstream oss;
    oss << "SURF keypoint count: " << keypoints.size();
    cv::Mat3b surf_vis_window = surf_vis(cv::Rect(0, surf_vis.rows - 30, 300, 30));
    surf_vis_window *= 0.2;
    cv::putText(surf_vis, oss.str(), cv::Point(20, surf_vis.rows - 10),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 200, 0), 1, CV_AA);
    
    cv::imshow("surf", surf_vis);
    oss.str("");
    oss << "surf" << setw(5) << setfill('0') << num << ".png";
    cv::imwrite(oss.str(), surf_vis);
    oss.str("");
    oss << "raw" << setw(5) << setfill('0') << num << ".png";
    cv::imwrite(oss.str(), prev3b);
    
    char key;
    key = cv::waitKey(ms);
    if(key == 'q')
      break;
    else if(key == ' ') {
      if(ms == 0)
        ms = 5;
      else
        ms = 0;
    }
  }
  
  return 0;
}
