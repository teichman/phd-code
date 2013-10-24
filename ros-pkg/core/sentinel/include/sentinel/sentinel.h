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
#include <sentinel/RecordingRequest.h>
#include <serializable/serializable.h>

class Sentinel : public OpenNI2Handler
{
public:
  Sentinel(double update_interval,
           double occupancy_threshold,
           int raytracing_threshold,
           double detection_threshold,
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

  //! Called before running processBackgroundSubtraction.
  virtual void processHook(openni::VideoFrameRef color) {}
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
  double detection_threshold_;
  HighResTimer update_timer_;
  cv::Mat3b vis_;
  bool visualize_;
  OpenNI2Interface oni_;
  cv::Mat3b color_;
  DepthMatPtr depth_;
  std::vector<uint32_t> indices_;
  std::vector<uint32_t> fg_markers_;
  std::vector<uint32_t> bg_fringe_markers_;

  void processRecording(openni::VideoFrameRef color);
  void processBackgroundSubtraction(openni::VideoFrameRef color,
                                    openni::VideoFrameRef depth,
                                    double sensor_timestamp,
                                    double wall_timestamp,
                                    size_t frame_id);
  void updateModel(openni::VideoFrameRef depth);
};

struct ImageSerializer
{
  void operator()(cv::Mat3b img, std::string path) { cv::imwrite(path, img); }
};

class ROSStreamingSentinel : public Sentinel
{
public:
  ROSStreamingSentinel(std::string sensor_id,
                       std::string recording_dir,
                       double update_interval,
                       double occupancy_threshold,
                       int raytracing_threshold,
                       double detection_threshold,
                       bool visualize,
                       OpenNI2Interface::Resolution color_res,
                       OpenNI2Interface::Resolution depth_res);

  ~ROSStreamingSentinel();
  
protected:
  std::string sensor_id_;
  //! Where to save recordings.  If "", don't save recordings.
  std::string recording_dir_;
  std::string frames_dir_;
  std::string tags_dir_;
  //! tag, time to stop recording at.
  std::map<std::string, ros::Time> recording_tags_;
  ThreadedSerializer<cv::Mat3b, ImageSerializer> serializer_;
  ros::NodeHandle nh_;
  ros::Publisher fg_pub_;
  ros::Publisher bg_pub_;
  ros::Subscriber rr_sub_;
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
  //! Used for processing recordings.
  void processHook(openni::VideoFrameRef color);
  void recordingRequestCallback(const sentinel::RecordingRequest& rr);
};

#endif // SENTINEL_H
