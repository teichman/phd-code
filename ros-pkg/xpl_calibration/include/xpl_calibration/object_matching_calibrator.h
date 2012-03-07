#ifndef OBJECT_MATCHING_CALIBRATOR_H
#define OBJECT_MATCHING_CALIBRATOR_H

#include <xpl_calibration/common.h>

class ReferenceObject
{
public:
  typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;
  typedef boost::shared_ptr<ReferenceObject> Ptr;
  
  rgbd::Cloud::ConstPtr pcd_;
  double timestamp_;
  Eigen::Vector3f centroid_;
  KdTree::Ptr tree_;
  
  ReferenceObject(rgbd::Cloud::ConstPtr pcd);
};

class FloatingObject
{
public:
  typedef boost::shared_ptr<FloatingObject> Ptr;
  
  rgbd::Cloud::Ptr pcd_;
  double timestamp_;
  Eigen::Vector3f centroid_;
  
  FloatingObject(rgbd::Cloud::Ptr pcd);
};

Eigen::Vector3f computeCentroid(const rgbd::Cloud& pcd);

class Correspondence
{
public:
  ReferenceObject::Ptr obj0_;
  FloatingObject::Ptr obj1_;
  
  Correspondence(ReferenceObject::Ptr obj0, FloatingObject::Ptr obj1);
  double computeLoss(double max_dist) const;
};

class CorrespondenceManager
{
public:
  typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;
  
  CorrespondenceManager(double dt_thresh, double centroid_thresh, double dist_thresh);
  //! Makes a kdtree.
  void addReferenceObject(rgbd::Cloud::ConstPtr pcd);
  void addFloatingObject(rgbd::Cloud::Ptr pcd);
  //! Recomputes correspondences.
  double computeLoss();
  //! Applies to clouds with id 1.
  void applyTimeOffset(double dt);
  //! Applies to clouds with id 1.
  //void applyTransform(const Eigen::Affine3f& transform);
  
protected:
  double dt_thresh_;
  double centroid_thresh_;
  double dist_thresh_;
  std::vector<ReferenceObject::Ptr> objects0_;
  std::vector<FloatingObject::Ptr> objects1_;
  std::vector<Correspondence> correspondences_;

  void computeCorrespondences();

};

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
    declareOutput<double>("Loss"); // Optimization objective after transform and dt optimization.
    declareOutput<const Eigen::Affine3f*>("RansacRefinedTransform");
    declareOutput<const Eigen::Affine3f*>("IcpRefinedTransform");
  }

  void compute();
  void debug() const;

protected:
  Eigen::Affine3f rough_transform_;
  Eigen::Affine3f ransac_refined_transform_;
  Eigen::Affine3f icp_refined_transform_;
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

  bool isAlmostIdentity(const Eigen::Affine3f& trans) const;
  rgbd::Cloud::Ptr visualizeInliers(const Eigen::Affine3f& transform) const;
  
};

#endif // OBJECT_MATCHING_CALIBRATOR_H
