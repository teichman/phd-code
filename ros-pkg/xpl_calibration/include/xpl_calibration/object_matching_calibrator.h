#ifndef OBJECT_MATCHING_CALIBRATOR_H
#define OBJECT_MATCHING_CALIBRATOR_H

#include <xpl_calibration/common.h>

class ObjectMatchingCalibrator : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > Objects;
  
  DECLARE_POD(ObjectMatchingCalibrator);
  ObjectMatchingCalibrator(std::string name) :
    Pod(name)
  {
    declareParam<double>("Threshold", 1.0); // Distance in meters between centroids to count as inliers.
    declareParam<int>("NumRansacIters", 100);
    declareParam<int>("NumCorrespondences", 5);

    declareInput<rgbd::Sequence::ConstPtr>("Sequence0");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence1");
    declareInput<const Objects*>("Objects0");
    declareInput<const Objects*>("Objects1");

    declareOutput<const Eigen::Affine3f*>("RoughTransform"); // Based on centroid matching only.
  }

  void compute();
  void debug() const;

protected:
  Eigen::Affine3f rough_transform_;

  void computeCentroids(const Objects& objects,
			std::vector< std::vector<Eigen::Vector3f> >* centroids) const;

  void sampleCorrespondence(const rgbd::Sequence& seq0,
			    const rgbd::Sequence& seq1,
			    const std::vector< std::vector<Eigen::Vector3f> >& centroids0,
			    const std::vector< std::vector<Eigen::Vector3f> >& centroids1,
			    pcl::TransformationFromCorrespondences* tfc) const;

  int countInliers(const Eigen::Affine3f& transform,
		   const std::vector< std::vector<Eigen::Vector3f> >& centroids0,
		   const std::vector< std::vector<Eigen::Vector3f> >& centroids1,
		   double thresh,
		   Eigen::Affine3f* refined_transform) const;
  
};

#endif // OBJECT_MATCHING_CALIBRATOR_H
