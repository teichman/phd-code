#ifndef XPL_CALIBRATOR_H
#define XPL_CALIBRATOR_H

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <bag_of_tricks/high_res_timer.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <bag_of_tricks/image_region_iterator.h>
#include <xpl_calibration/plane_finder.h>

class TransformValidator
{
public:
  //! tar to ref
  std::vector<Eigen::Affine3f> candidates_;
  rgbd::Cloud::ConstPtr ref_pcd_;
  rgbd::Cloud::ConstPtr tar_pcd_;
  pcl::PointCloud<pcl::Normal>::ConstPtr ref_normals_;
  pcl::PointCloud<pcl::Normal>::ConstPtr tar_normals_;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ref_tree_;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tar_tree_;
  
  TransformValidator(double gamma = 0.1);
  Eigen::Affine3f compute();
  double computeLoss(const rgbd::Cloud& ref,
		     const pcl::PointCloud<pcl::Normal>& ref_normals,
		     pcl::search::KdTree<pcl::PointXYZRGB>& ref_tree,
		     const rgbd::Cloud& tar) const;
  
  void fineTuneAlignment(const rgbd::Cloud& ref,
			 pcl::search::KdTree<pcl::PointXYZRGB>& ref_tree,
			 const pcl::PointCloud<pcl::Normal>& ref_normals,
			 const rgbd::Cloud& tar,
			 Eigen::Affine3f* transform) const;

protected:
  //! Weighting of the color difference term.
  double gamma_;
};

#endif // XPL_CALIBRATOR_H
