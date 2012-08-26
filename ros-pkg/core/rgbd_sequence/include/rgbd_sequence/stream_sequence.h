#ifndef STREAM_SEQUENCE_H
#define STREAM_SEQUENCE_H

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
#include <XnDevice.h>

namespace rgbd
{

  class StreamSequence
  {
  public:
    typedef boost::shared_ptr<StreamSequence> Ptr;
    typedef boost::shared_ptr<const StreamSequence> ConstPtr;
  
    std::vector<std::string> img_names_;
    std::vector<std::string> dpt_names_;
    std::vector<std::string> clk_names_;
    std::vector<double> timestamps_; //Keep these in memory
    std::string root_path_;
    PrimeSenseModel model_;
    XnDeviceDefinition wtf_;

    //! Does not initialize anything.
    StreamSequence();
    //! Creates a new directory at root_path for streaming to.
    void init(const std::string& root_path);
    //! Loads existing model and timestamps at root_path_, prepares for streaming from here.
    void load(const std::string& root_path);
    //! Saves PrimeSenseModel and timestamps to root_path_.
    //! Must have an initialized model_.
    void save() const;
    size_t size() const;
    void writeFrame(const Frame& frame);
    //! Loads from disk and fills frame.
    void readFrame(size_t idx, Frame* frame) const;
    //! Returns the nearest frame, no matter how far away it is in time.  Check dt to find out.
    void readFrame(double timestamp, double* dt, Frame* frame) const;
    //! Adds dt to all timestamps.  Does not save.
    void applyTimeOffset(double dt);

  protected:
    //! dt is signed.
    size_t seek(double timestamp, double* dt) const;

    // Assignment op & copy constructor would deep copy if they were implemented.
    StreamSequence(const StreamSequence& seq);
    StreamSequence& operator=(const StreamSequence& seq);
  };


  inline bool isFinite(const Point& pt)
  {
    return (pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z));
  }

}

#endif // STREAM_SEQUENCE_H


