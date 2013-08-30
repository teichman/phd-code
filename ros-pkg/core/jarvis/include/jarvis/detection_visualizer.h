#ifndef DETECTION_VISUALIZER_H
#define DETECTION_VISUALIZER_H

#include <iostream>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sentinel/Detection.h>

class DetectionVisualizer
{
public:
  DetectionVisualizer(int width, int height);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  cv::Mat3b vis_;

  void callback(const sentinel::Detection& msg);
};

#endif // DETECTION_VISUALIZER_H
