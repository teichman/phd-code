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
    grabber_.reset (new pcl::ImageGrabber<pcl::PointXYZRGBA> (dir, 0, false, true) ); // PCLZF mode
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
    pcl::PointCloud <pcl::PointXYZRGBA>::ConstPtr cloud = grabber_->at (idx);
    // Make img_ and depth_;

    frame->img_ = cv::Mat3b::zeros (model_.height_, model_.width_);
    frame->depth_ = DepthMatPtr(new DepthMat (model_.height_, model_.width_));

    for (int y = 0; y < cloud->height; y++)
    {
      for (int x = 0; x < cloud->width; x++)
      {
        const pcl::PointXYZRGBA &pt = cloud->at (x, y);
        frame->img_ (y, x)[0] = pt.b;
        frame->img_ (y, x)[1] = pt.g;
        frame->img_ (y, x)[2] = pt.r;
        frame->depth_->coeffRef (y, x) = (pt.z > max_depth_ ? 0 : pt.z * 1000);
      }
    }
  }

  void StreamSequencePCLWrapper::readFrame(double timestamp, double* dt, Frame* frame) const
  {
    size_t idx = seek(timestamp, dt);
    readFrame(idx, frame);    
  }

  size_t StreamSequencePCLWrapper::seek(double timestamp, double* dt) const
  {
    ROS_ASSERT(!timestamps_.empty());
    
    // TODO: This could be much faster than linear search.
    size_t nearest = 0;
    *dt = numeric_limits<double>::max();
    for(size_t i = 0; i < timestamps_.size(); ++i) {
      double d = timestamp - timestamps_[i];
      if(fabs(d) < *dt) {
        *dt = d;
        nearest = i;
      }
    }

    return nearest;
  }

  size_t StreamSequencePCLWrapper::size() const
  {
    return grabber_->size ();
  }
  
} // namespace rgbd



