#ifndef DETECTION_VISUALIZER_H
#define DETECTION_VISUALIZER_H

#include <iostream>
#include <deque>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <timer/timer.h>
#include <asp/asp.h>
#include <sentinel/Detection.h>

class DetectionVisualizer
{
public:
  DetectionVisualizer(int width, int height);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  cv::Mat3b color_vis_;
  cv::Mat3b depth_vis_;
  HighResTimer hrt_;
  std::deque<double> timestamps_;
  asp::Asp asp_;

  void callback(const sentinel::Detection& msg);
};

#endif // DETECTION_VISUALIZER_H
