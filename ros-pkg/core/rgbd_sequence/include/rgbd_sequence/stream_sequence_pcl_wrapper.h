#ifndef STREAM_SEQUENCE_PCL_WRAPPER_H
#define STREAM_SEQUENCE_PCL_WRAPPER_H

#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/image_grabber.h>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>
#include <timer/timer.h>
#include <rgbd_sequence/primesense_model.h>
#include <rgbd_sequence/stream_sequence_base.h>

namespace rgbd
{

  class StreamSequencePCLWrapper : public StreamSequenceBase
  {
  public:
    typedef boost::shared_ptr<StreamSequencePCLWrapper> Ptr;
    typedef boost::shared_ptr<const StreamSequencePCLWrapper> ConstPtr;
    
    using StreamSequenceBase::timestamps_;
    using StreamSequenceBase::model_;
  
    //! The maximum depth in meters, used when reading data.
    //! Anything beyond this is set to 0.
    double max_depth_;

    //! Does not initialize anything.
    StreamSequencePCLWrapper();
    //! Loads existing model and timestamps at root_path_, prepares for streaming from here.


    size_t size() const;
  protected:
    //! Loads from disk and fills frame.
    void readFrameImpl(size_t idx, Frame* frame) const;



  protected:
    void loadImpl(const std::string& root_path);

    boost::shared_ptr <pcl::ImageGrabber<pcl::PointXYZRGB> > grabber_;
    
  };

}

#endif // STREAM_SEQUENCE_PCL_WRAPPER_H



