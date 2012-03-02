#include <xpl_calibration/xpl_calibrator.h>

using namespace std;

ColorWheel::ColorWheel(int num_colors) :
  colors_(vector<cv::Vec3b>(num_colors))
{
  for(size_t i = 0; i < colors_.size(); ++i)
    colors_[i] = cv::Vec3b(rand() % 255, rand() % 255, rand() % 255);
}

cv::Vec3b ColorWheel::getColor(size_t num) const
{
  assert(num < colors_.size());
  return colors_[num];
}

