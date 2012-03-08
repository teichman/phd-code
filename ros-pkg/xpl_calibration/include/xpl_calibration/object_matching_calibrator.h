#ifndef OBJECT_MATCHING_CALIBRATOR_H
#define OBJECT_MATCHING_CALIBRATOR_H

#include <xpl_calibration/common.h>
#include <optimization/optimization.h>
#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>

class ObjectMatchingCalibrator : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > ObjectClouds;
  typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;

  DECLARE_POD(ObjectMatchingCalibrator);
  ObjectMatchingCalibrator(std::string name) :
    Pod(name)
  {
    declareParam<double>("CentroidThreshold", 0.5); // Distance in meters between centroids to count as inliers.
    declareParam<double>("DistanceThreshold", 0.1); // Maximum distance for hinge loss in objective function.
    declareParam<double>("TimeOffsetRange", 0.1);
    declareParam<double>("TimeOffsetResolution", 0.005);
    declareParam<double>("TimeCorrespondenceThreshold", 0.03);
    declareParam<int>("NumRansacIters", 1000);
    declareParam<int>("NumCorrespondences", 3);

    declareInput<rgbd::Sequence::ConstPtr>("Sequence0");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence1");
    declareInput<const ObjectClouds*>("Objects0");
    declareInput<const ObjectClouds*>("Objects1");

    declareOutput<const Eigen::Affine3f*>("RansacRoughTransform"); // Based on centroid matching only.
    declareOutput<double>("SyncOffset");
    declareOutput<const Eigen::Affine3f*>("RansacRefinedTransform");
    declareOutput<const Eigen::Affine3f*>("IcpRefinedTransform");
    declareOutput<const Eigen::Affine3f*>("GridSearchTransform");
  }

  void compute();
  void debug() const;

protected:
  Eigen::Affine3f rough_transform_;
  Eigen::Affine3f ransac_refined_transform_;
  Eigen::Affine3f icp_refined_transform_;
  Eigen::Affine3f gridsearch_transform_;
  std::vector<Eigen::VectorXd> gs_history_;
  std::vector< std::vector<Eigen::Vector3f> > centroids0_;
  std::vector< std::vector<Eigen::Vector3f> > centroids1_;

  void computeCentroids(const ObjectClouds& objects,
			std::vector< std::vector<Eigen::Vector3f> >* centroids) const;

  void sampleCorrespondence(const rgbd::Sequence& seq0,
			    const rgbd::Sequence& seq1,
			    const std::vector< std::vector<Eigen::Vector3f> >& centroids0,
			    const std::vector< std::vector<Eigen::Vector3f> >& centroids1,
			    pcl::TransformationFromCorrespondences* tfc) const;

  int countInliers(const Eigen::Affine3f& transform,
		   const std::vector< std::vector<Eigen::Vector3f> >& centroids0,
		   const std::vector< std::vector<Eigen::Vector3f> >& centroids1,
		   double ransac_thresh,
		   Eigen::Affine3f* refined_transform = NULL,
		   std::vector<rgbd::Cloud::ConstPtr>* inlier_clouds0 = NULL,
		   std::vector<rgbd::Cloud::ConstPtr>* inlier_clouds1 = NULL,
		   std::vector<Eigen::Vector3f>* inlier_centroids0 = NULL,
		   std::vector<Eigen::Vector3f>* inlier_centroids1 = NULL) const;

    
  Eigen::Affine3f alignInlierModels(const std::vector< std::vector<Eigen::Vector3f> >& centroids0,
				    const std::vector< std::vector<Eigen::Vector3f> >& centroids1) const;

  rgbd::Cloud::Ptr visualizeInliers(const Eigen::Affine3f& transform) const;
  
};

// class LossFunction : public ScalarFunction
// {
// public:
//   LossFunction(aoeu);
//   double eval(const Eigen::VectorXd& x) const;
// };


//! Rotations are in radians.
Eigen::Affine3f generateTransform(double rx, double ry, double rz,
				  double tx, double ty, double tz);


#endif // OBJECT_MATCHING_CALIBRATOR_H
