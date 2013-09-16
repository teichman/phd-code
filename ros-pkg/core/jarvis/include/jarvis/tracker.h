#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <timer/timer.h>
#include <sentinel/Foreground.h>
#include <sentinel/Background.h>

struct Blob
{
  typedef boost::shared_ptr<Blob> Ptr;
  typedef boost::shared_ptr<const Blob> ConstPtr;
  
  uint64_t frame_id_;
  double sensor_timestamp_;
  ros::Time wall_timestamp_;
  std::vector<uint32_t> indices_;
  std::vector<uint16_t> depth_;
  //! RGB.
  std::vector<uint8_t> color_;
};

class Tracker
{
public:
  bool visualize_;
  cv::Mat1f depth_;
  cv::Mat1b foreground_;
  cv::Mat1i blobs_;
  std::map< size_t, std::vector<Blob::Ptr> > tracks_;

  Tracker(size_t max_track_length);
  void update(sentinel::ForegroundConstPtr msg);

protected:
  size_t max_track_length_;
  size_t next_track_id_;

  void reconstructForeground(sentinel::Foreground::ConstPtr msg,
                             cv::Mat1f depth, cv::Mat1b foreground) const;
};

void displayBlobs(const std::vector<Blob::ConstPtr>& blobs, cv::Mat3b img);

#endif // TRACKER_H
