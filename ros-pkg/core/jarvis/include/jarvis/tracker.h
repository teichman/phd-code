#ifndef TRACKER_H
#define TRACKER_H

#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <online_learning/common.h>
#include <serializable/serializable.h>
#include <sentinel/Foreground.h>
#include <jarvis/discrete_bayes_filter.h>

class Blob : public Serializable
{
public:
  typedef boost::shared_ptr<Blob> Ptr;
  typedef boost::shared_ptr<const Blob> ConstPtr;
  
  uint64_t frame_id_;
  //! In seconds.
  double sensor_timestamp_;
  ros::Time wall_timestamp_;
  int width_;
  int height_;
  std::vector<uint32_t> indices_;
  //! RGB.
  std::vector<uint8_t> color_;
  std::vector<float> depth_;

  //! idx indexes into indices_.
  void coords(size_t idx, int* u, int* v) const;
  
  //! Fills cloud_, centroid_, and kdtree_ from the indices_, color_, and depth_ data.
  //! You'll have to redo this if you save and then load.
  void project(bool compute_kdtree = true);
  void clearProjected();
  
  boost::shared_ptr<Cloud> cloud_;
  boost::shared_ptr<KdTree> kdtree_;
  Eigen::Vector3f centroid_;

  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  //! Reconstructs an image showing the object.  Variable size.
  cv::Mat3b image() const;
  //! Reconstructs an image showing the object.  Variable size.
  cv::Mat3b depthImage() const;
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
  uint64_t frame_id_;
  //! track ID, most recent Blob.
  std::map<size_t, Blob::Ptr> tracks_;

  Tracker(size_t max_track_length);
  void update(sentinel::ForegroundConstPtr msg);
  //! Fills img with a representation of the current state of the tracks.
  //void draw(cv::Mat3b img) const;
  //! Fills img with a representation of the current state of the tracks.
  //! Track id colors or track classification colors.
  void draw(cv::Mat3b img, bool track_classification_colors = false,
            const std::map<size_t, DiscreteBayesFilter>& filters = std::map<size_t, DiscreteBayesFilter>()) const;
  cv::Mat3b draw() const;
  
protected:
  size_t max_track_length_;
  size_t next_track_id_;
  boost::posix_time::ptime ptime_;

  void reconstructForeground(sentinel::ForegroundConstPtr msg,
                             cv::Mat1f depth, cv::Mat1b foreground) const;
  // Projects the blobs if necessary.
  double distance(Blob& prev, Blob& curr) const;
};

void displayBlobs(const std::vector<Blob::ConstPtr>& blobs, cv::Mat3b img);
//! rotation must be one of 0, 90, 180, or 270.
void orient(int rotation, cv::Mat3b* img);
void addTimestamp(const boost::posix_time::ptime& ptime, cv::Mat3b img);

#endif // TRACKER_H
