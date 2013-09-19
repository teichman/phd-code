#ifndef JARVIS_H
#define JARVIS_H

#include <iostream>
#include <deque>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <timer/timer.h>
#include <sentinel/reconstructor.h>
#include <jarvis/tracker.h>

class Jarvis
{
public:
  Jarvis(int vis_level);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber fg_sub_;
  ros::Subscriber bg_sub_;
  cv::Mat3b color_vis_;
  cv::Mat3b depth_vis_;
  Reconstructor reconstructor_;
  Tracker tracker_;
  int vis_level_;

  void foregroundCallback(sentinel::ForegroundConstPtr msg);
  void backgroundCallback(sentinel::BackgroundConstPtr msg);
};

#endif // JARVIS_H
