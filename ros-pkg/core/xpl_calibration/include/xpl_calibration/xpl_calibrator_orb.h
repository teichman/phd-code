#ifndef XPL_CALIBRATOR_ORB_H
#define XPL_CALIBRATOR_ORB_H

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
#include <xpl_calibration/descriptor_database.h>
#include <xpl_calibration/transform_validator.h>

class XplCalibratorOrb
{
public:
  XplCalibratorOrb();
  //! Computes transform that will move target to reference.
  //! T * target = reference.
  Eigen::Affine3f calibrate(rgbd::Sequence::ConstPtr reference,
                            rgbd::Sequence::ConstPtr target) const;

  pcl::PointXYZRGB samplePoint(const std::vector<cv::KeyPoint>& keypoints,
                               const rgbd::Cloud& pcd, int* idx) const;
  pcl::PointXYZRGB getPoint(const cv::KeyPoint& keypoint, const rgbd::Cloud& pcd) const;
};

#endif // XPL_CALIBRATOR_ORB_H
