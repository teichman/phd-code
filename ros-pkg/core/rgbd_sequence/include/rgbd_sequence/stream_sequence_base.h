#ifndef STREAM_SEQUENCE_BASE_H
#define STREAM_SEQUENCE_BASE_H

#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>
#include <timer/timer.h>
#include <rgbd_sequence/primesense_model.h>

namespace rgbd
{

  class StreamSequenceBase
  {
  public:
    typedef boost::shared_ptr<StreamSequenceBase> Ptr;
    typedef boost::shared_ptr<const StreamSequenceBase> ConstPtr;
  
    std::vector<double> timestamps_;
    PrimeSenseModel model_;

    virtual ~StreamSequenceBase () {}

    //! Loads existing model and timestamps at root_path_, prepares for streaming from here.
    virtual void load(const std::string& root_path) = 0;
    //! Saves PrimeSenseModel and timestamps to root_path_.
    //! Must have an initialized model_.
    virtual size_t size() const = 0;

    virtual void readFrame(size_t idx, Frame* frame) const = 0;
    //! Returns the nearest frame, no matter how far away it is in time.  Check dt to find out.
    virtual void readFrame(double timestamp, double* dt, Frame* frame) const = 0;

    //! dt is signed.
    size_t seek(double timestamp, double* dt) const;
    
  protected:
  };


  inline bool isFinite(const Point& pt)
  {
    return (pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z));
  }

}

#endif // STREAM_SEQUENCE_BASE_H



