#ifndef DETECTION_VISUALIZER_H
#define DETECTION_VISUALIZER_H

#include <iostream>
#include <deque>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <timer/timer.h>
#include <asp/asp.h>
#include <sentinel/reconstructor.h>

class DetectionVisualizer
{
public:
  DetectionVisualizer(int width, int height);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber fg_sub_;
  ros::Subscriber bg_sub_;
  cv::Mat3b color_vis_;
  cv::Mat3b depth_vis_;
  HighResTimer hrt_;
  std::deque<double> timestamps_;
  asp::Asp asp_;
  Reconstructor reconstructor_;

  void foregroundCallback(sentinel::ForegroundConstPtr msg);
  void backgroundCallback(sentinel::BackgroundConstPtr msg);
};

#endif // DETECTION_VISUALIZER_H
