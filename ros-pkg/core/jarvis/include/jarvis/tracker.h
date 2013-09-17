#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <timer/timer.h>
#include <sentinel/Foreground.h>
#include <sentinel/Background.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Cloud;
//typedef pcl::search::KdTree<pcl::PointXYZRGB> KdTree;
typedef pcl::KdTreeFLANN<pcl::PointXYZRGB> KdTree;

struct Blob
{
  typedef boost::shared_ptr<Blob> Ptr;
  typedef boost::shared_ptr<const Blob> ConstPtr;
  
  uint64_t frame_id_;
  double sensor_timestamp_;
  ros::Time wall_timestamp_;
  int width_;
  int height_;
  std::vector<uint32_t> indices_;
  //! RGB.
  std::vector<uint8_t> color_;
  std::vector<float> depth_;

  Cloud::Ptr cloud_;
  KdTree::Ptr kdtree_;
  Eigen::Vector4f centroid_;

  //! Fills cloud_, centroid_, and kdtree_ from the indices_, color_, and depth_ data.
  void project();
};

//! Takes FG messages, outputs Blobs with track ids.
//! Creates & deletes tracks when necessary.
class Tracker
{
public:
  bool visualize_;
  cv::Mat1f depth_;
  cv::Mat3b color_;
  cv::Mat1b foreground_;
  cv::Mat1i assignments_;
  std::vector< std::vector<int> > indices_;
  //! track ID, most recent Blob.
  std::map<size_t, Blob::Ptr> tracks_;

  Tracker(size_t max_track_length);
  void update(sentinel::ForegroundConstPtr msg);

protected:
  size_t max_track_length_;
  size_t next_track_id_;

  void reconstructForeground(sentinel::Foreground::ConstPtr msg,
                             cv::Mat1f depth, cv::Mat1b foreground) const;
  double distance(const Blob& prev, const Blob& curr) const;
};

void displayBlobs(const std::vector<Blob::ConstPtr>& blobs, cv::Mat3b img);

#endif // TRACKER_H
