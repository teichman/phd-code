#ifndef DESCRIPTOR_PIPELINE_H
#define DESCRIPTOR_PIPELINE_H

#include <iostream>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <timer/timer.h>
#include <eigen_extensions/eigen_extensions.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

class DescriptorPipeline
{
public:
  DescriptorPipeline(YAML::Node params, cv::Rect roi);
  int dimensionality() const;
  Eigen::VectorXd computeDescriptors(cv::Mat3b img);
  
protected:
  YAML::Node params_;
  cv::Rect roi_;
  cv::Mat3b cropped_;
  cv::Mat1b gray_;
  cv::Mat1b diff_;  // The difference image.
  cv::Mat1b prev_gray_;

  double SURFCountDescriptor(cv::Mat3b img) const;
  double differenceDescriptor(cv::Mat1b diff) const;
};

#endif // DESCRIPTOR_PIPELINE_H
