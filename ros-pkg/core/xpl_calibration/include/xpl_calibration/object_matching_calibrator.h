#ifndef OBJECT_MATCHING_CALIBRATOR_H
#define OBJECT_MATCHING_CALIBRATOR_H

#include <xpl_calibration/common.h>
#include <optimization/grid_search.h>
#include <Eigen/Geometry>
#include <pcl/visualization/cloud_viewer.h>

#define TRANSFORM_SEARCH_METHOD (getenv("TRANSFORM_SEARCH_METHOD") ? std::string(getenv("TRANSFORM_SEARCH_METHOD")) : std::string("GridSearch"))

typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;

class ObjectMatchingCalibrator;
class LossFunction : public ScalarFunction
{
public:
  LossFunction(const std::vector<KdTree::Ptr>& trees0,
               const std::vector<rgbd::Cloud::ConstPtr>& pcds0,
               const std::vector<rgbd::Cloud::Ptr>& pcds1,
               const pipeline::Params& params);
  double eval(const Eigen::VectorXd& x) const;
  double computeLoss(KdTree::Ptr tree0, const rgbd::Cloud& pcd0, const rgbd::Cloud& pcd1) const;

  bool use_fsv_; //SDM Unprotected so this could be set to false
protected:
  std::vector<KdTree::Ptr> trees0_;
  std::vector<rgbd::Cloud::ConstPtr> pcds0_;
  std::vector<rgbd::Cloud::Ptr> pcds1_;
  double dt_thresh_;
  double max_dist_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;

  int projectPoint(const rgbd::Cloud& pcd, const rgbd::Point& pt,
                   int* u, int* v) const;
};

class ObjectMatchingCalibrator : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > ObjectClouds;


  DECLARE_POD(ObjectMatchingCalibrator);
  ObjectMatchingCalibrator(std::string name) :
    Pod(name)
  {
    declareParam<int>("NumRansacIters", 1000);
    declareParam<int>("NumCorrespondences", 3);
    declareParam<double>("CentroidThreshold", 0.5); // Distance in meters between centroids to count as inliers.
    declareParam<double>("TimeCorrespondenceThreshold", 0.015);
    declareParam<double>("Downsampling", 0.9); // Drop this fraction.  0.0 means using all the data, 1.0 none.
    declareParam<double>("DistanceThreshold", 0.3); // Maximum distance for hinge loss in objective function.
    declareParam<double>("TransformThreshold", 0.01);
    declareParam<std::string>("TransformSearchMethod", TRANSFORM_SEARCH_METHOD); // "GridSearch" or "ICP"
    declareParam<int>("MaxConsecutiveFrames", 10);
    declareParam<int>("MaxFrames", 100);
    declareParam<int>("MinObjectSize", 1000);
    
    // Camera intrinsics for sequence 0.
    // These should come from the corresponding StreamSequence members.
    declareParam<double>("Seq0Fx");
    declareParam<double>("Seq0Fy");
    declareParam<double>("Seq0Cx");
    declareParam<double>("Seq0Cy");


    declareInput<Stream::ConstPtr>("Sequence0");
    declareInput<Stream::ConstPtr>("Sequence1");
    declareInput<const ObjectClouds*>("Objects0");
    declareInput<const ObjectClouds*>("Objects1");

    declareOutput<const Eigen::Affine3f*>("CentroidRansacTransform");
    declareOutput<const Eigen::Affine3f*>("FinalTransform");
    declareOutput<double>("SyncOffset");
  }

  void compute();

protected:
  double sync_;
  Eigen::Affine3f ransac_transform_;
  Eigen::Affine3f transform_;
  Eigen::Affine3f final_transform_;
  int outer_iter_;

  void findGoodFrames (std::vector<size_t> &frames);

  void accumulateObjects(const ObjectClouds& objects,
                         const std::vector<size_t> &frames,
                         std::vector<rgbd::Cloud::Ptr>* pcds) const;
  void getData(const std::vector<size_t> &frames,
               std::vector<KdTree::Ptr>* trees0,
               std::vector<rgbd::Cloud::ConstPtr>* pcds0,
               std::vector<rgbd::Cloud::Ptr>* pcds1) const;

  void checkInput() const;
  Eigen::Affine3f centroidRansac(const std::vector<size_t> &frames) const;
  rgbd::Cloud::ConstPtr getNearestReferenceObject(const rgbd::Cloud& pcd1) const;
  void getSyncedInlierModels(const Eigen::Affine3f& transform,
                             double sync,
                             std::vector<rgbd::Cloud::ConstPtr>* inliers0,
                             std::vector<rgbd::Cloud::Ptr>* inliers1) const;
    
  void computeCentroids(const ObjectClouds& objects,
                        const std::vector<size_t> &frames,
                        std::vector< std::vector<Eigen::Vector3f> >* centroids) const;

  void sampleCorrespondence(const Stream& seq0,
                            const Stream& seq1,
                            const std::vector<size_t> &frames,
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

  //! Moves pcds1 by the returned incremental transform.
  Eigen::Affine3f updateTransformICP(const std::vector<KdTree::Ptr>& trees0,
                                     const std::vector<rgbd::Cloud::ConstPtr>& pcds0,
                                     const std::vector<rgbd::Cloud::Ptr>& pcds1) const;
  //! Moves pcds1 by the returned incremental transform.
  Eigen::Affine3f updateTransformGS(const std::vector<KdTree::Ptr>& trees0,
                                    const std::vector<rgbd::Cloud::ConstPtr>& pcds0,
                                    const std::vector<rgbd::Cloud::Ptr>& pcds1) const;
  double updateSync(const std::vector<KdTree::Ptr>& trees0,
                    const std::vector<rgbd::Cloud::ConstPtr>& pcds0,
                    const std::vector<rgbd::Cloud::Ptr>& pcds1) const;
      
  void visualizeInliers(const std::string& name, const Eigen::Affine3f& transform, const std::vector<size_t> &frames) const;
  void visualizeResult(const std::string& name, const Eigen::Affine3f& transform, double sync) const;
  void visualizeScenes(const std::string& name,
                       const std::vector<rgbd::Cloud::ConstPtr>& pcds0,
                       const std::vector<rgbd::Cloud::Ptr>& pcds1) const;
  
  void downsampleAndTransform(const std::vector<rgbd::Cloud::Ptr>& source,
                              const Eigen::Affine3f& transform,
                              std::vector<rgbd::Cloud::Ptr>* destination) const;

  double gridSearchSync(ScalarFunction::Ptr lf) const;
  Eigen::Affine3f gridSearchTransform(ScalarFunction::Ptr lf) const;
};

         
int seek(const std::vector<rgbd::Cloud::ConstPtr>& pcds0,
         double ts1, double dt_thresh);
int seek(const Stream& pcds0,
         double ts1, double dt_thresh);
int seek(const std::vector<rgbd::Cloud::Ptr>& pcds0,
         double ts1, double dt_thresh);
//! Rotations are in radians.
Eigen::Affine3f generateTransform(double rx, double ry, double rz,
                                  double tx, double ty, double tz);
//! Inverse of generateTransform
void generateXYZYPR(const Eigen::Affine3f &trans, 
    double &rx, double &ry, double &rz, double &tx, double &ty, double &tz);

#endif // OBJECT_MATCHING_CALIBRATOR_H


