#ifndef OBJECT_MATCHING_CALIBRATOR_H
#define OBJECT_MATCHING_CALIBRATOR_H

#include <xpl_calibration/common.h>
#include <optimization/grid_search.h>
#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;


class SyncLossFunction : public ScalarFunction
{
public:
  SyncLossFunction(const std::vector<KdTree::Ptr>& trees0,
		 const std::vector<rgbd::Cloud::ConstPtr>& scenes0,
		   const std::vector<rgbd::Cloud::Ptr>& scenes1,
		   double dt_thresh,
		   double max_dist);
  double eval(const Eigen::VectorXd& x) const;
  double computeLoss(KdTree::Ptr tree0, const rgbd::Cloud& pcd0, const rgbd::Cloud& pcd1) const;

protected:
  std::vector<KdTree::Ptr> trees0_;
  std::vector<rgbd::Cloud::ConstPtr> scenes0_;
  std::vector<rgbd::Cloud::Ptr> scenes1_;
  double dt_thresh_;
  double max_dist_;
};

class ObjectMatchingCalibrator : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > ObjectClouds;


  DECLARE_POD(ObjectMatchingCalibrator);
  ObjectMatchingCalibrator(std::string name) :
    Pod(name)
  {
    declareParam<bool>("UseGridSearch", false); // TODO: Factor this out into another node.
    declareParam<double>("CentroidThreshold", 0.5); // Distance in meters between centroids to count as inliers.
    declareParam<double>("DistanceThreshold", 0.3); // Maximum distance for hinge loss in objective function.
    declareParam<double>("TimeOffsetRange", 0.1);
    declareParam<double>("TimeOffsetResolution", 0.005);
    declareParam<double>("TimeCorrespondenceThreshold", 0.015);
    declareParam<double>("ICPDownsampling", 0.0); // Drop this fraction.  0.0 means using all the data, 1.0 none.
    declareParam<double>("ICPThreshold", 0.001);
    declareParam<double>("GridSearchDownsampling", 0.95); // Drop this fraction.  0.0 means using all the data, 1.0 none.
    declareParam<int>("NumRansacIters", 1000);
    declareParam<int>("NumCorrespondences", 3);

    declareInput<rgbd::Sequence::ConstPtr>("Sequence0");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence1");
    declareInput<const ObjectClouds*>("Objects0");
    declareInput<const ObjectClouds*>("Objects1");

    declareOutput<double>("SyncOffset");
    declareOutput<const Eigen::Affine3f*>("CentroidRansacTransform");
    declareOutput<const Eigen::Affine3f*>("IcpTransform");
    declareOutput<const Eigen::Affine3f*>("GridSearchTransform");
    declareOutput<const Eigen::Affine3f*>("FinalTransform");
  }

  void compute();

protected:
  double sync_;
  Eigen::Affine3f ransac_transform_;
  Eigen::Affine3f icp_transform_;
  Eigen::Affine3f final_transform_;
  std::vector<Eigen::VectorXd> gs_history_;

  void extractScenes(const ObjectClouds& objects,
		     std::vector<rgbd::Cloud::Ptr>* scenes) const;
  void getScenes(std::vector<KdTree::Ptr>* trees0,
		 std::vector<rgbd::Cloud::ConstPtr>* scenes0,
		 std::vector<rgbd::Cloud::Ptr>* scenes1) const;

  void checkInput() const;
  Eigen::Affine3f centroidRansac() const;
  rgbd::Cloud::ConstPtr getNearestReferenceObject(const rgbd::Cloud& pcd1) const;
  void getSyncedInlierModels(const Eigen::Affine3f& transform,
			     double sync,
			     std::vector<rgbd::Cloud::ConstPtr>* inliers0,
			     std::vector<rgbd::Cloud::Ptr>* inliers1) const;
    
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


  Eigen::Affine3f updateICP(const std::vector<KdTree::Ptr>& trees0,
			    const std::vector<rgbd::Cloud::ConstPtr>& scenes0,
			    const std::vector<rgbd::Cloud::Ptr>& scenes1) const;
  double updateSync(const std::vector<KdTree::Ptr>& trees0,
		    const std::vector<rgbd::Cloud::ConstPtr>& scenes0,
		    const std::vector<rgbd::Cloud::Ptr>& scenes1) const;
      
  void visualizeInliers(const std::string& name, const Eigen::Affine3f& transform) const;
  void visualizeTransform(const std::string& name, const Eigen::Affine3f& transform) const;

  void gridSearch(const std::vector<rgbd::Cloud::Ptr>& pcds0,
		  const std::vector<rgbd::Cloud::Ptr>& pcds1,
		  Eigen::Affine3f* final_transform,
		  double* final_sync) const;

  void downsampleAndTransform(const std::vector<rgbd::Cloud::Ptr>& source,
			      const Eigen::Affine3f& transform,
			      std::vector<rgbd::Cloud::Ptr>* destination) const;

  double gridSearchSync(ScalarFunction::Ptr lf) const;
};

int seek(const std::vector<rgbd::Cloud::ConstPtr>& scenes0,
	 double ts1, double dt_thresh);
	 


// //! Rotations are in radians.
// Eigen::Affine3f generateTransform(double rx, double ry, double rz,
// 				  double tx, double ty, double tz);

// int projectPoint(const rgbd::Cloud& pcd, const rgbd::Point& pt,
// 		 int* u, int* v);

#endif // OBJECT_MATCHING_CALIBRATOR_H


