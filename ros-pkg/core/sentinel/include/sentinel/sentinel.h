#ifndef SENTINEL_H
#define SENTINEL_H

#include <ctime>
#include <queue>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bag_of_tricks/high_res_timer.h>
#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <sentinel/background_model.h>
#include <sentinel/Detection.h>

class Sentinel : public OpenNI2Handler
{
public:
  Sentinel(double update_interval,
           int max_training_imgs,
           double threshold,
           bool visualize,
           OpenNI2Interface::Resolution color_res,
           OpenNI2Interface::Resolution depth_res);
  virtual ~Sentinel()
  {
    #if JARVIS_DEBUG
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    #endif
  }
  
  void rgbdCallback(openni::VideoFrameRef color, openni::VideoFrameRef depth);
  void run();
  virtual void handleDetection(openni::VideoFrameRef color, openni::VideoFrameRef depth,
                               const std::vector<uint8_t>& mask,
                               size_t num_in_mask, double timestamp) = 0;
                               

protected:
  boost::shared_ptr<BackgroundModel> model_;
  std::queue<openni::VideoFrameRef> training_;
  double update_interval_;
  int max_training_imgs_;
  HighResTimer update_timer_;
  cv::Mat3b vis_;
  double threshold_;
  std::vector<uint8_t> mask_;
  bool visualize_;
  OpenNI2Interface oni_;
  cv::Mat3b color_;
  DepthMatPtr depth_;
  
  void process(openni::VideoFrameRef color,
               openni::VideoFrameRef depth,
               double ts);
  void updateModel(openni::VideoFrameRef depth);
};

class DiskStreamingSentinel : public Sentinel
{
public:
  DiskStreamingSentinel(std::string dir,
                        double save_interval,
                        double update_interval,
                        int max_training_imgs,
                        double threshold,
                        bool visualize,
                        OpenNI2Interface::Resolution color_res,
                        OpenNI2Interface::Resolution depth_res);
  ~DiskStreamingSentinel()
  {
    #if JARVIS_DEBUG
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    #endif
  }

  void handleDetection(openni::VideoFrameRef color,
                       openni::VideoFrameRef depth,
                       const std::vector<uint8_t>& mask,
                       size_t num_in_mask, double timestamp);

protected:
  std::string dir_;
  double save_interval_;
  HighResTimer save_timer_;
  
  void save(cv::Mat3b color, DepthMatConstPtr depth, cv::Mat3b vis, double ts) const;
  cv::Mat1b depthMatToCV(const DepthMat& depth) const;
};

class ROSStreamingSentinel : public Sentinel
{
public:
  ROSStreamingSentinel(double update_interval,
                       int max_training_imgs,
                       double threshold,
                       bool visualize,
                       OpenNI2Interface::Resolution color_res,
                       OpenNI2Interface::Resolution depth_res);

  void handleDetection(openni::VideoFrameRef color,
                       openni::VideoFrameRef depth,
                       const std::vector<uint8_t>& mask,
                       size_t num_in_mask, double timestamp);

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  sentinel::Detection msg_;
};

#endif // SENTINEL_H
