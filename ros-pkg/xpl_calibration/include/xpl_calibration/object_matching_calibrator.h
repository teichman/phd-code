#ifndef OBJECT_MATCHING_CALIBRATOR_H
#define OBJECT_MATCHING_CALIBRATOR_H

#include <xpl_calibration/common.h>

class ObjectMatchingCalibrator : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > Objects;
  typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;

  DECLARE_POD(ObjectMatchingCalibrator);
  ObjectMatchingCalibrator(std::string name) :
    Pod(name)
  {
    declareParam<double>("Threshold", 0.50); // Distance in meters between centroids to count as inliers.
    declareParam<int>("NumRansacIters", 1000);
    declareParam<int>("NumCorrespondences", 3);

    declareInput<rgbd::Sequence::ConstPtr>("Sequence0");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence1");
    declareInput<const Objects*>("Objects0");
    declareInput<const Objects*>("Objects1");

    declareOutput<const Eigen::Affine3f*>("RoughTransform"); // Based on centroid matching only.
    declareOutput<const Eigen::Affine3f*>("RefinedTransform"); // Alignment of object models.
  }

  void compute();
  void debug() const;

protected:
  Eigen::Affine3f rough_transform_;
  Eigen::Affine3f refined_transform_;

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
		   Eigen::Affine3f* refined_transform = NULL,
		   std::vector<rgbd::Cloud::ConstPtr>* inliers0 = NULL,
		   std::vector<rgbd::Cloud::ConstPtr>* inliers1 = NULL) const;
    
  Eigen::Affine3f alignInlierModels(const std::vector< std::vector<Eigen::Vector3f> >& centroids0,
				    const std::vector< std::vector<Eigen::Vector3f> >& centroids1) const;

  bool isAlmostIdentity(const Eigen::Affine3f& trans) const;
    
};

#endif // OBJECT_MATCHING_CALIBRATOR_H
