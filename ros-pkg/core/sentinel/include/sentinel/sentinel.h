#ifndef SENTINEL_H
#define SENTINEL_H

#include <ctime>
#include <queue>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <timer/timer.h>
#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <sentinel/background_model.h>
#include <sentinel/Foreground.h>
#include <sentinel/Background.h>

class Sentinel : public OpenNI2Handler
{
public:
  Sentinel(double update_interval,
           double occupancy_threshold,
           int raytracing_threshold,
           bool visualize,
           OpenNI2Interface::Resolution color_res,
           OpenNI2Interface::Resolution depth_res);
  virtual ~Sentinel()
  {
    #if JARVIS_DEBUG
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    #endif
  }
  
  void rgbdCallback(openni::VideoFrameRef color, openni::VideoFrameRef depth,
                    size_t frame_id, double timestamp);
  void run();
  virtual void handleDetection(openni::VideoFrameRef color,
                               openni::VideoFrameRef depth,
                               const std::vector<uint32_t>& indices,
                               const std::vector<uint32_t>& fg_markers,
                               const std::vector<uint32_t>& bg_fringe_markers,
                               double sensor_timestamp,
                               double wall_timestamp,
                               size_t frame_id) = 0;
  virtual void handleNonDetection(openni::VideoFrameRef color,
                                  openni::VideoFrameRef depth,
                                  double sensor_timestamp,
                                  double wall_timestamp,
                                  size_t frame_id) {}
  void debug(int x, int y) { model_->debug(x, y); }
                               

protected:
  boost::shared_ptr<BackgroundModel> model_;
  std::queue<openni::VideoFrameRef> training_;
  double update_interval_;
  double occupancy_threshold_;
  HighResTimer update_timer_;
  cv::Mat3b vis_;
  bool visualize_;
  OpenNI2Interface oni_;
  cv::Mat3b color_;
  DepthMatPtr depth_;
  std::vector<uint32_t> indices_;
  std::vector<uint32_t> fg_markers_;
  std::vector<uint32_t> bg_fringe_markers_;
  
  void process(openni::VideoFrameRef color,
               openni::VideoFrameRef depth,
               double sensor_timestamp,
               double wall_timestamp,
               size_t frame_id);
  void updateModel(openni::VideoFrameRef depth);
};

class ROSStreamingSentinel : public Sentinel
{
public:
  ROSStreamingSentinel(std::string sensor_id,
                       double update_interval,
                       double occupancy_threshold,
                       int raytracing_threshold,
                       bool visualize,
                       OpenNI2Interface::Resolution color_res,
                       OpenNI2Interface::Resolution depth_res);

protected:
  std::string sensor_id_;
  ros::NodeHandle nh_;
  ros::Publisher fg_pub_;
  ros::Publisher bg_pub_;
  sentinel::Foreground fgmsg_;
  sentinel::Background bgmsg_;
  int bg_index_x_;
  int bg_index_y_;

  void initializeBackgroundMessage();
  void initializeForegroundMessage();
  void handleDetection(openni::VideoFrameRef color,
                       openni::VideoFrameRef depth,
                       const std::vector<uint32_t>& indices,
                       const std::vector<uint32_t>& fg_markers,
                       const std::vector<uint32_t>& bg_fringe_markers,
                       double sensor_timestamp,
                       double wall_timestamp,
                       size_t frame_id);
  
  void handleNonDetection(openni::VideoFrameRef color,
                          openni::VideoFrameRef depth,
                          double sensor_timestamp,
                          double wall_timestamp,
                          size_t frame_id);
};

// class DiskStreamingSentinel : public Sentinel
// {
// public:
//   DiskStreamingSentinel(std::string dir,
//                         double save_interval,
//                         double update_interval,
//                         int max_training_imgs,
//                         bool visualize,
//                         OpenNI2Interface::Resolution color_res,
//                         OpenNI2Interface::Resolution depth_res);
//   ~DiskStreamingSentinel()
//   {
//     #if JARVIS_DEBUG
//     std::cout << __PRETTY_FUNCTION__ << std::endl;
//     #endif
//   }

//   void handleDetection(openni::VideoFrameRef color,
//                        openni::VideoFrameRef depth,
//                        const std::vector<uint8_t>& mask,
//                        size_t num_in_mask,
//                        double sensor_timestamp,
//                        double wall_timestamp,
//                        size_t frame_id);

// protected:
//   std::string dir_;
//   double save_interval_;
//   HighResTimer save_timer_;
  
//   void save(cv::Mat3b color, DepthMatConstPtr depth, cv::Mat3b vis, double ts) const;
//   cv::Mat1b depthMatToCV(const DepthMat& depth) const;
// };

#endif // SENTINEL_H
