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

  cv::Mat3b orig;
  for(int i = 0; i < 100; ++i) cap >> orig;

  int num = 0;
  while(true) {
    cap >> orig;
    if(num % 100 == 0) {
      ostringstream oss;
      oss << "img" << setw(5) << setfill('0') << num << ".png";
      cv::imwrite(oss.str(), orig);
    }
    ++num;
  }

  return 0;
}

