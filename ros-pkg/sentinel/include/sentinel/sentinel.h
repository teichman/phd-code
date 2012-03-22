#ifndef SENTINEL_H
#define SENTINEL_H

#include <queue>
#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>
#include <bag_of_tricks/high_res_timer.h>
#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/stream_recorder.h>
#include <sentinel/background_model.h>

class Sentinel
{
public:
  Sentinel(double update_interval,
	   int max_training_imgs,
	   double threshold,
	   const std::string& device_id = "",
	   pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz);
  void rgbdCallback(const boost::shared_ptr<openni_wrapper::Image>& rgb,
		    const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
		    float f_inv);
  void run();

protected:
  pcl::OpenNIGrabber grabber_;
  BackgroundModel model_;
  std::queue<rgbd::DepthMatConstPtr> training_;
  double update_interval_;
  int max_training_imgs_;
  HighResTimer hrt_;
  cv::Mat3b vis_;
  double threshold_;
  
  void process(rgbd::DepthMatConstPtr depth, cv::Mat3b img);
  void updateModel(rgbd::DepthMatConstPtr depth);
};

#endif // SENTINEL_H
