#ifndef ORB_MATCHER_H
#define ORB_MATCHER_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <ros/console.h>
#include <pipeline/pod.h>
#include <xpl_calibration/descriptor_database.h>
#include <rgbd_sequence/rgbd_sequence.h>

class OrbMatcher : public pipeline::Pod
{
public:
  typedef Eigen::Matrix<uchar, 32, Eigen::Dynamic> PackedDescriptors;
  typedef boost::shared_ptr<const PackedDescriptors> PackedDescriptorsConstPtr;
  typedef std::vector<Eigen::Affine3f> Transforms;
  typedef const std::vector<Eigen::Affine3f>* TransformsConstPtr;
  typedef const std::vector<cv::KeyPoint>* KeyPointsConstPtr;
  
  DECLARE_POD(OrbMatcher);
  OrbMatcher(std::string name) :
    Pod(name)
  {
    declareParam<int>("NumSamples", 100000);
    declareParam<double>("DistanceThresh", 0.04);
    
    declareInput<rgbd::Cloud::ConstPtr>("Cloud0");
    declareInput<rgbd::Cloud::ConstPtr>("Cloud1");
    declareInput<cv::Mat3b>("Image0");
    declareInput<cv::Mat3b>("Image1");
    declareInput<KeyPointsConstPtr>("Keypoints0");
    declareInput<KeyPointsConstPtr>("Keypoints1");
    declareInput<PackedDescriptorsConstPtr>("Descriptors0");
    declareInput<PackedDescriptorsConstPtr>("Descriptors1");
    
    declareOutput<TransformsConstPtr>("Transforms");
  }

  void compute();
  void debug() const;

protected:
  std::vector<Eigen::Affine3f> transforms_;
  std::vector< std::vector<int> > matches_;
  
  pcl::PointXYZRGB getPoint(const cv::KeyPoint& keypoint, const rgbd::Cloud& pcd) const;
};


#endif // ORB_MATCHER_H
