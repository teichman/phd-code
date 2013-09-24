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
  //! rotation is the angle in degrees and must be one of 0, 90, 180, or 270.
  Jarvis(int vis_level, int rotation = 0);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber fg_sub_;
  ros::Subscriber bg_sub_;
  cv::Mat3b color_vis_;
  cv::Mat3b depth_vis_;
  Reconstructor reconstructor_;
  Tracker tracker_;
  int vis_level_;
  int rotation_;

  void foregroundCallback(sentinel::ForegroundConstPtr msg);
  void backgroundCallback(sentinel::BackgroundConstPtr msg);
  void orient(int rotation, cv::Mat3b img) const;
};

#endif // JARVIS_H
