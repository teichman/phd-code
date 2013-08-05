#ifndef SENTINEL_H
#define SENTINEL_H

#include <ctime>
#include <queue>
#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>
#include <bag_of_tricks/high_res_timer.h>
#include <sentinel/background_model.h>

class Sentinel
{
public:
  Sentinel(std::string name,
           double update_interval,
           double save_interval,
           int max_training_imgs,
           double threshold,
           const std::string& device_id = "",
           pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);
  void rgbdCallback(const boost::shared_ptr<openni_wrapper::Image>& rgb,
                    const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
                    float f_inv);
  void run();

protected:
  pcl::OpenNIGrabber grabber_;
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
  
  void process(DepthMatConstPtr depth, cv::Mat3b img, double ts);
  void updateModel(DepthMatConstPtr depth);
  void save(DepthMatConstPtr depth, cv::Mat3b img, cv::Mat3b vis, double ts) const;
  cv::Mat1b depthMatToCV(const DepthMat& depth) const;
};

#endif // SENTINEL_H
