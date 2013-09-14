#ifndef TRACKER_H
#define TRACKER_H

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
  std::map< size_t, std::vector<Blob::Ptr> > tracks_;

  Tracker(size_t max_track_length);
  void update(sentinel::ForegroundConstPtr msg);

protected:
  size_t max_track_length_;
  size_t next_track_id_;
};

void displayBlobs(const std::vector<Blob::ConstPtr>& blobs, cv::Mat3b img);

#endif // TRACKER_H
