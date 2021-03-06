#ifndef ORB_MATCHER_H
#define ORB_MATCHER_H

#include <xpl_calibration/common.h>

class OrbMatcher : public pl::Pod
{
public:
  typedef Eigen::Matrix<uchar, 32, Eigen::Dynamic> PackedDescriptors;
  typedef boost::shared_ptr<const PackedDescriptors> PackedDescriptorsConstPtr;
  typedef std::vector<Eigen::Affine3f> Transforms;
  typedef const std::vector<Eigen::Affine3f>* TransformsConstPtr;
  typedef const std::vector<cv::KeyPoint>* KeyPointsConstPtr;
  
  DECLARE_POD(OrbMatcher);
  OrbMatcher(std::string name) :
    Pod(name),
    keypoint_cloud0_(new rgbd::Cloud),
    keypoint_cloud1_(new rgbd::Cloud)
  {
    declareParam<int>("NumSamples", 100000);
    declareParam<double>("DistanceThresh", 0.04);
    declareParam<double>("MinInlierPercent", 0.1); // [0, 1]
    
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
  rgbd::Cloud::Ptr keypoint_cloud0_;
  rgbd::Cloud::Ptr keypoint_cloud1_;
  rgbd::Cloud transformed_keypoint_cloud1_;
  std::vector<Eigen::Affine3f> pruned_;
  
  pcl::PointXYZRGB getPoint(const cv::KeyPoint& keypoint, const rgbd::Cloud& pcd) const;
};


#endif // ORB_MATCHER_H
