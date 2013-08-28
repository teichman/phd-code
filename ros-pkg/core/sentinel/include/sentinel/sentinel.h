#ifndef SENTINEL_H
#define SENTINEL_H

#include <ctime>
#include <queue>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bag_of_tricks/high_res_timer.h>
#include <sentinel/background_model.h>
#include <openni2_interface/openni2_interface.h>

class Sentinel : public OpenNI2Handler
{
public:
  Sentinel(double update_interval,
           int max_training_imgs,
           double threshold,
           bool visualize,
           OpenNI2Interface::Resolution resolution);
  virtual ~Sentinel()
  {
    #if JARVIS_DEBUG
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    #endif
  }
  
  void rgbdCallback(const openni::VideoFrameRef& color,
                    const openni::VideoFrameRef& depth);
  void run();
  virtual void handleDetection(cv::Mat3b color, DepthMatConstPtr depth,
                               cv::Mat1b mask, double timestamp) = 0;
                               

protected:
  BackgroundModel::Ptr model_;
  std::queue<DepthMatConstPtr> training_;
  double update_interval_;
  int max_training_imgs_;
  HighResTimer update_timer_;
  cv::Mat3b vis_;
  double threshold_;
  cv::Mat1b mask_;
  bool visualize_;
  OpenNI2Interface oni_;
  cv::Mat3b color_;
  DepthMatPtr depth_;
  
  void process(cv::Mat3b color, DepthMatConstPtr depth, double ts);
  void updateModel(DepthMatConstPtr depth);
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
                        OpenNI2Interface::Resolution res);
  ~DiskStreamingSentinel()
  {
    #if JARVIS_DEBUG
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    #endif
  }
  
  void handleDetection(cv::Mat3b color, DepthMatConstPtr depth,
                       cv::Mat1b mask, double timestamp);

protected:
  std::string dir_;
  double save_interval_;
  HighResTimer save_timer_;
  
  void save(cv::Mat3b color, DepthMatConstPtr depth, cv::Mat3b vis, double ts) const;
  cv::Mat1b depthMatToCV(const DepthMat& depth) const;
};

// class ROSStreamingSentinel : public Sentinel
// {
// };

#endif // SENTINEL_H
