#include <nutcracker/descriptor_pipeline.h>

using namespace std;
using namespace Eigen;

DescriptorPipeline::DescriptorPipeline(YAML::Node params, cv::Rect roi) :
  params_(params),
  roi_(roi)
{
}

int DescriptorPipeline::dimensionality() const
{
  return 6;
}

Eigen::VectorXd DescriptorPipeline::computeDescriptors(cv::Mat3b img)
{
  cv::Mat(img, roi_).copyTo(cropped_);
  cv::cvtColor(cropped_, gray_, CV_BGR2GRAY);

  // If this is the first time we're running, indicate that we can't
  // compute descriptors yet because we need a previous image.
  if(prev_gray_.rows != gray_.rows) {
    prev_gray_ = gray_;
    return VectorXd();
  }

  if(diff_.rows != gray_.rows)
    diff_ = cv::Mat1b(gray_.size(), 0);
  for(int y = 0; y < gray_.rows; ++y)
    for(int x = 0; x < gray_.cols; ++x)
      diff_(y, x) = fabs((float)gray_(y, x) - prev_gray_(y, x));

  prev_gray_ = gray_;

  VectorXd x(dimensionality());
  x(0) = 1;
  x(1) = SURFCountDescriptor(cropped_);
  x(2) = x(1) * x(1);
  x(3) = differenceDescriptor(diff_);
  x(4) = x(3) * x(3);
  x(5) = x(1) * x(3);

  return x;
}

double DescriptorPipeline::differenceDescriptor(cv::Mat1b diff) const 
{
  uint8_t thresh = params_["difference-image-threshold"].as<float>();
  double val = 0;
  for(int y = 0; y < diff.rows; ++y)
    for(int x = 0; x < diff.cols; ++x)
      if(diff(y, x) > thresh)
        ++val;
  return val / (diff.rows * diff.cols);
}

double DescriptorPipeline::SURFCountDescriptor(cv::Mat3b img) const
{
  cv::SURF surf(params_["surf-hessian-threshold"].as<double>(),
                params_["surf-num-octaves"].as<int>(),
                params_["surf-num-octave-layers"].as<int>());
  vector<cv::KeyPoint> keypoints;
  surf(img, cv::Mat(), keypoints);
  return keypoints.size();
}
