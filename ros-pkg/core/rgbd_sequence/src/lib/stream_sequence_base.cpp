#include <rgbd_sequence/stream_sequence_base.h>
#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/stream_sequence_pcl_wrapper.h>
#include <limits>

rgbd::StreamSequenceBase::Ptr rgbd::StreamSequenceBase::initializeFromDirectory (const std::string &dir)
{
  // Check if it's the old or new format
  StreamSequenceBase::Ptr out;
  if (boost::filesystem::exists (dir + "/primesense_model"))
    out.reset (new StreamSequence);
  else
    out.reset (new StreamSequencePCLWrapper);
  out->load (dir);
  return out;
}

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
  
void rgbd::StreamSequenceBase::applyTimeOffset(double dt)
{
  for(size_t i = 0; i < timestamps_.size(); ++i)
    timestamps_[i] += dt;
}
  
rgbd::Cloud::Ptr rgbd::StreamSequenceBase::getCloud(size_t idx) const
{
  Cloud::Ptr pcd(new Cloud);
  Frame frame;
  readFrame(idx, &frame);
  model_.frameToCloud(frame, pcd.get());
  return pcd;
}

rgbd::Cloud::Ptr rgbd::StreamSequenceBase::getCloud(double timestamp, double* dt) const
{
  Cloud::Ptr pcd(new Cloud);
  Frame frame;
  readFrame(timestamp, dt, &frame);
  model_.frameToCloud(frame, pcd.get());
  return pcd;
}

cv::Mat3b rgbd::StreamSequenceBase::getImage(size_t idx) const
{
  Frame frame;
  readFrame(idx, &frame);
  return frame.img_;
}

cv::Mat3b rgbd::StreamSequenceBase::getImage(double timestamp, double* dt) const
{
  Frame frame;
  readFrame(timestamp, dt, &frame);
  return frame.img_;
}
