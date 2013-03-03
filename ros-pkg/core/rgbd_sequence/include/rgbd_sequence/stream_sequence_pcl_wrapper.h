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

namespace rgbd
{

  class StreamSequencePCLWrapper
  {
  public:
    typedef boost::shared_ptr<StreamSequencePCLWrapper> Ptr;
    typedef boost::shared_ptr<const StreamSequencePCLWrapper> ConstPtr;
  
    //! The maximum depth in meters, used when reading data.
    //! Anything beyond this is set to 0.
    double max_depth_;
    //! Primesense model. Will be initialized once the grabber starts
    PrimeSenseModel model_;
    //! Timestamps. Will be loaded from grabber
    std::vector<double> timestamps_;

    //! Does not initialize anything.
    StreamSequencePCLWrapper();
    //! Loads existing model and timestamps at root_path_, prepares for streaming from here.
    void load(const std::string& root_path);

    size_t size() const;
    //! Loads from disk and fills frame.
    void readFrame(size_t idx, Frame* frame) const;
    //! Returns the nearest frame, no matter how far away it is in time.  Check dt to find out.
    void readFrame(double timestamp, double* dt, Frame* frame) const;

    //! dt is signed.
    size_t seek(double timestamp, double* dt) const;

  protected:
    boost::shared_ptr <pcl::ImageGrabber<pcl::PointXYZRGBA> > grabber_;
    
  };

}

#endif // STREAM_SEQUENCE_PCL_WRAPPER_H



