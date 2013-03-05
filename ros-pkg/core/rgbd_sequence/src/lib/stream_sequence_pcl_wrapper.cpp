#include <rgbd_sequence/stream_sequence_pcl_wrapper.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  StreamSequencePCLWrapper::StreamSequencePCLWrapper():
    max_depth_(numeric_limits<double>::max())
  {
  }
  
  void StreamSequencePCLWrapper::load(const std::string& dir)
  {
    grabber_.reset (new pcl::ImageGrabber<pcl::PointXYZRGB> (dir, 0, false, true) ); // PCLZF mode
    grabber_->start ();
    // Update primesense model
    model_.id_ = 0; // TODO ?
    model_.width_ = 640; // TODO ?
    model_.height_ = 480; // TODO ?
    // model_.weights?
    grabber_->getCameraIntrinsics (model_.fx_, model_.fy_, model_.cx_, model_.cy_);
    // Update timestamps
    timestamps_.resize (grabber_->size ());
    for (size_t i = 0; i < grabber_->size (); i++)
    {
      uint64_t timestamp_microsec;
      grabber_->getTimestampAtIndex (i, timestamp_microsec);
      timestamps_[i] = static_cast<double> (timestamp_microsec) * 1E-6;
    }
  }

  void StreamSequencePCLWrapper::readFrame(size_t idx, Frame* frame) const
  {
    ROS_ASSERT (idx < grabber_->size ());
    rgbd::Cloud::ConstPtr cloud = grabber_->at (idx);
    // Make img_ and depth_;
    model_.cloudToFrame (*cloud, frame);
    frame->timestamp_ = timestamps_[idx];
  }

  void StreamSequencePCLWrapper::readFrame(double timestamp, double* dt, Frame* frame) const
  {
    size_t idx = seek(timestamp, dt);
    readFrame(idx, frame);    
  }

  size_t StreamSequencePCLWrapper::size() const
  {
    return grabber_->size ();
  }
  
} // namespace rgbd



