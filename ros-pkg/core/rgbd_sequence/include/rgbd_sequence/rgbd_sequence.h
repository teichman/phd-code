#ifndef RGBD_SEQUENCE_H
#define RGBD_SEQUENCE_H

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
#include <ros/assert.h>

namespace rgbd
{

  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> Cloud;

  class Sequence : public Serializable
  {
  public:
    typedef boost::shared_ptr<Sequence> Ptr;
    typedef boost::shared_ptr<const Sequence> ConstPtr;
  
    std::vector<cv::Mat3b> imgs_;
    std::vector<Cloud::Ptr> pcds_;

    Sequence();
    //! Deep-copies.
    Sequence(const Sequence& seq);
    //! Deep-copies.
    Sequence& operator=(const Sequence& seq);
    void save(const std::string& filename) const;
    void load(const std::string& filename);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    size_t size() const { ROS_ASSERT(imgs_.size() == pcds_.size()); return imgs_.size(); }
  };

  //! path can point to a directory of sequences or a single sequence.
  //! sequences does not have to be empty.
  void loadSequences(const std::string& path,
		     std::vector<Sequence::Ptr>* sequences);
  void zthresh(rgbd::Cloud::Ptr pcd, double max_z);
}



#endif // RGBD_SEQUENCE_H
