#ifndef RGBD_SEQUENCE_H
#define RGBD_SEQUENCE_H

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <serializable/serializable.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> RGBDCloud;
  
class RGBDSequence : public Serializable
{
public:
  typedef boost::shared_ptr<RGBDSequence> Ptr;
  typedef boost::shared_ptr<const RGBDSequence> ConstPtr;
  
  std::vector<cv::Mat3b> imgs_;
  std::vector<RGBDCloud::Ptr> pcds_;

  RGBDSequence();
  //! Deep-copies.
  RGBDSequence(const RGBDSequence& seq);
  //! Deep-copies.
  RGBDSequence& operator=(const RGBDSequence& seq);
  void save(const std::string& filename) const;
  void load(const std::string& filename);
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
};

//! path can point to a directory of sequences or a single sequence.
//! sequences does not have to be empty.
void loadSequences(const std::string& path,
		   std::vector<RGBDSequence::Ptr>* sequences);

#endif // RGBD_SEQUENCE_H
