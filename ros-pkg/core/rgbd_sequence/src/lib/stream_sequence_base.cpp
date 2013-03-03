#include <rgbd_sequence/stream_sequence_base.h>
#include <limits>

size_t rgbd::StreamSequenceBase::seek(double timestamp, double* dt) const
{
  ROS_ASSERT(!timestamps_.empty());
  
  // TODO: This could be much faster than linear search.
  size_t nearest = 0;
  *dt = std::numeric_limits<double>::max();
  for(size_t i = 0; i < timestamps_.size(); ++i) {
    double d = timestamp - timestamps_[i];
    if(fabs(d) < *dt) {
      *dt = d;
      nearest = i;
    }
  }

  return nearest;
}
