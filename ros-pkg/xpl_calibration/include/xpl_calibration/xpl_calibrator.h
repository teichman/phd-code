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
#include <bag_of_tricks/image_region_iterator.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <xpl_calibration/plane_finder.h>

class Junction
{
public:
  //! Reference point in plane 1.
  Eigen::Vector3f pt1_;
  //! Reference point in plane 2.
  Eigen::Vector3f pt2_;
  //! Reference point at the intesection of both planes.
  Eigen::Vector3f pt3_;
  //! Average surface normal of plane 1.
  Eigen::Vector3f normal1_;
  //! Average surface normal of plane 2.
  Eigen::Vector3f normal2_;
  //! Direction of the crease.
  Eigen::Vector3f creasedir_;
  //! Average color of plane 1.
  Eigen::Vector3f color1_;
  //! Average color of plane 2.
  Eigen::Vector3f color2_;
  //! Max component in the direction of creasedir_, for both planes.
  double max_;
  //! Min component in the direction of creasedir_, for both planes.
  double min_;
  cv::Point2i img_centroid1_;
  cv::Point2i img_centroid2_;

  //! Swaps pt1 and pt1. Updates everything else appropriately.
  void swap();
};

std::ostream& operator<<(const Junction& junc, std::ostream& out);

class XplCalibrator
{
public:
  XplCalibrator();
  //! Computes transform that will move target to reference.
  //! T * target = reference.
  Eigen::Affine3f calibrate(rgbd::Sequence::ConstPtr reference,
			    rgbd::Sequence::ConstPtr target) const;

protected:
  double distance_thresh_;
  //! Min angle between adjacent planes to use them.
  double min_angle_;
  //! Weighting of the color difference term.
  double gamma_;
  //! Step size to use in searching over the translation.
  double granularity_;

  double computeLoss(const rgbd::Cloud& ref,
		     const pcl::PointCloud<pcl::Normal>& ref_normals,
		     pcl::search::KdTree<pcl::PointXYZRGB>& ref_tree,
		     const rgbd::Cloud& tar) const;
  
  bool computeTransform(const Junction& ref,
			const Junction& tar,
			Eigen::Affine3f* transform) const;
  
  void findJunctions(const rgbd::Cloud& pcd,
		     const pcl::PointCloud<pcl::Normal>& normals,
		     std::vector<Junction>* junctions) const;

  void applyTranslation(const rgbd::Cloud& src,
			const Eigen::Vector3f& translation,
			rgbd::Cloud* dst) const;

  cv::Mat3b visualizeJunctions(const std::vector<Junction>& junctions,
			       cv::Mat3b img) const;

  void fineTuneAlignment(const rgbd::Cloud& ref,
			 pcl::search::KdTree<pcl::PointXYZRGB>& ref_tree,
			 const pcl::PointCloud<pcl::Normal>& ref_normals,
			 const rgbd::Cloud& tar,
			 Eigen::Affine3f* transform) const;
    
};

#endif // XPL_CALIBRATOR_H
