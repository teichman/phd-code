#ifndef STREAM_SEQUENCE_H
#define STREAM_SEQUENCE_H

#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/assert.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>
#include <timer/timer.h>
#include <rgbd_sequence/primesense_model.h>
#include <rgbd_sequence/stream_sequence_base.h>

namespace rgbd
{

  class StreamSequence : public StreamSequenceBase
  {
  public:
    typedef boost::shared_ptr<StreamSequence> Ptr;
    typedef boost::shared_ptr<const StreamSequence> ConstPtr;

    using StreamSequenceBase::timestamps_;
    using StreamSequenceBase::model_;
    using StreamSequenceBase::seek;
  
    std::vector<std::string> img_names_;
    std::vector<std::string> dpt_names_;
    std::vector<std::string> clk_names_;
    std::string root_path_;
    //! The maximum depth in meters, used when reading data.
    //! Anything beyond this is set to 0.
    double max_depth_;

    //! Does not initialize anything.
    StreamSequence();
    //! Creates a new directory at root_path for streaming to.
    void init(const std::string& root_path);
    //! Saves PrimeSenseModel and timestamps to root_path_.
    //! Must have an initialized model_.
    void save() const;
    size_t size() const;
    void writeFrame(const Frame& frame);

    
  protected:
    //! Loads existing model and timestamps at root_path_, prepares for streaming from here.
    void loadImpl(const std::string& root_path);
    // Assignment op & copy constructor would deep copy if they were implemented.
    //! Loads from disk and fills frame.
    void readFrameImpl(size_t idx, Frame* frame) const;

    StreamSequence(const StreamSequence& seq);
    StreamSequence& operator=(const StreamSequence& seq);
  };


}

#endif // STREAM_SEQUENCE_H


