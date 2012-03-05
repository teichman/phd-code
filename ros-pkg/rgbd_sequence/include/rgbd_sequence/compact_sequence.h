#ifndef COMPACT_SEQUENCE_H
#define COMPACT_SEQUENCE_H

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
#include <Eigen/Eigen>

namespace rgbd
{

  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> Cloud;
  typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> DepthMat;

  class CompactSequence : public Serializable
  {
  public:
    typedef boost::shared_ptr<CompactSequence> Ptr;
    typedef boost::shared_ptr<const CompactSequence> ConstPtr;
  
    std::vector<cv::Mat3b> imgs_;
    std::vector<DepthMat> depths_;
    std::vector<double> timestamps_;
    double focal_length_;

    CompactSequence();
    //! Deep-copies.
    CompactSequence(const CompactSequence& seq);
    //! Deep-copies.
    CompactSequence& operator=(const CompactSequence& seq);
    void save(const std::string& filename) const;
    void load(const std::string& filename);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    size_t size() const { ROS_ASSERT(imgs_.size() == depths_.size()); return imgs_.size(); }
    Cloud::Ptr getCloud(size_t frame) const;
  };


}

#endif // COMPACT_SEQUENCE_H

