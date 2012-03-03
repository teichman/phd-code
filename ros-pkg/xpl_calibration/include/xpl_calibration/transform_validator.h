#ifndef XPL_CALIBRATOR_H
#define XPL_CALIBRATOR_H

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pipeline/pod.h>
#include <bag_of_tricks/high_res_timer.h>
#include <bag_of_tricks/image_region_iterator.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <xpl_calibration/plane_finder.h>

class TransformValidator : public pipeline::Pod
{
public:
  typedef std::vector<Eigen::Affine3f> Candidates;
  typedef const std::vector<Eigen::Affine3f>* CandidatesConstPtr;
  
  DECLARE_POD(TransformValidator);
  TransformValidator(std::string name) :
    Pod(name)
  {
    declareParam<double>("Gamma", 0.1); // Weighting of the color difference term.
    declareInput<CandidatesConstPtr>("Candidates");
    declareInput<rgbd::Cloud::ConstPtr>("Cloud0");
    declareInput<rgbd::Cloud::ConstPtr>("Cloud1");
    declareOutput<const Eigen::Affine3f*>("BestTransform");
  }

  void compute();
  void debug() const;
  
protected:
  //! 0 to 1
  std::vector<Eigen::Affine3f> candidates_;
  rgbd::Cloud::ConstPtr cloud0_;
  rgbd::Cloud::ConstPtr cloud1_;
  pcl::PointCloud<pcl::Normal>::ConstPtr normals0_;
  pcl::PointCloud<pcl::Normal>::ConstPtr normals1_;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree0_;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1_;

  double computeLoss(const rgbd::Cloud& cloud0,
		     const pcl::PointCloud<pcl::Normal>& normals0,
		     pcl::search::KdTree<pcl::PointXYZRGB>& tree0,
		     const rgbd::Cloud& cloud1) const;
  
  void fineTuneAlignment(const rgbd::Cloud& cloud0,
			 pcl::search::KdTree<pcl::PointXYZRGB>& tree0,
			 const pcl::PointCloud<pcl::Normal>& normals0,
			 const rgbd::Cloud& cloud1,
			 Eigen::Affine3f* transform) const;

};

#endif // XPL_CALIBRATOR_H
