#ifndef OBJECT_MATCHING_CALIBRATOR_H
#define OBJECT_MATCHING_CALIBRATOR_H

#include <xpl_calibration/common.h>
#include <optimization/grid_search.h>
#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;

class LossFunction : public ScalarFunction
{
public:
  LossFunction(double max_dist,
	       double dt_thresh,
	       const std::vector<rgbd::Cloud::Ptr>& pcds0,
	       const std::vector<rgbd::Cloud::Ptr>& pcds1);
  
  double eval(const Eigen::VectorXd& x);

protected:
  double max_dist_;
  double dt_thresh_;
  std::vector<KdTree::Ptr> trees0_;
  std::vector<rgbd::Cloud::Ptr> pcds0_;
  std::vector<rgbd::Cloud::Ptr> pcds1_;
  
  double computeLoss(KdTree::Ptr tree0, const rgbd::Cloud& pcd0,
		     const rgbd::Cloud& pcd1, const Eigen::Affine3f& transform) const;
  int seek(double ts1) const;
};


class ObjectMatchingCalibrator : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > ObjectClouds;


  DECLARE_POD(ObjectMatchingCalibrator);
  ObjectMatchingCalibrator(std::string name) :
    Pod(name)
  {
    declareParam<double>("CentroidThreshold", 0.5); // Distance in meters between centroids to count as inliers.
    declareParam<double>("DistanceThreshold", 0.1); // Maximum distance for hinge loss in objective function.
    declareParam<double>("TimeOffsetRange", 0.1);
    declareParam<double>("TimeOffsetResolution", 0.005);
    declareParam<double>("TimeCorrespondenceThreshold", 0.015);
    declareParam<double>("Downsampling", 0.9); // Drop this fraction.  0.0 means using all the data.
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

  void gridSearch(const std::vector<rgbd::Cloud::Ptr>& pcds0,
		  const std::vector<rgbd::Cloud::Ptr>& pcds1,
		  Eigen::Affine3f* final_transform,
		  double* final_sync) const;

  void downsampleAndTransform(const std::vector<rgbd::Cloud::Ptr>& source,
			      const Eigen::Affine3f& transform,
			      std::vector<rgbd::Cloud::Ptr>* destination) const;

  Eigen::Affine3f gridSearchTransform(LossFunction::Ptr lf) const;
  double gridSearchSync(LossFunction::Ptr lf) const;
    

  
};


//! Rotations are in radians.
Eigen::Affine3f generateTransform(double rx, double ry, double rz,
				  double tx, double ty, double tz);

int projectPoint(const rgbd::Cloud& pcd, const rgbd::Point& pt,
		 int* u, int* v);

#endif // OBJECT_MATCHING_CALIBRATOR_H
