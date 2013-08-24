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
  Sentinel(std::string name,
           double update_interval,
           double save_interval,
           int max_training_imgs,
           double threshold,
           bool visualize,
           OpenNI2Interface::Resolution res);
  void rgbdCallback(const openni::VideoFrameRef& color,
                    const openni::VideoFrameRef& depth);
  void run();

protected:
  OpenNI2Interface oni_;
  BackgroundModel model_;
  std::queue<DepthMatConstPtr> training_;
  double update_interval_;
  double save_interval_;
  int max_training_imgs_;
  HighResTimer update_timer_;
  HighResTimer save_timer_;
  cv::Mat3b vis_;
  double threshold_;
  std::string dir_;
  cv::Mat1b mask_;
  bool visualize_;
  
  void process(DepthMatConstPtr depth, cv::Mat3b img, double ts);
  void updateModel(DepthMatConstPtr depth);
  void save(DepthMatConstPtr depth, cv::Mat3b img, cv::Mat3b vis, double ts) const;
  cv::Mat1b depthMatToCV(const DepthMat& depth) const;
};

#endif // SENTINEL_H
