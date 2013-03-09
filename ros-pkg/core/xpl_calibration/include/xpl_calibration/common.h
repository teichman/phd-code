#ifndef XPL_CALIBRATION_PODS_H
#define XPL_CALIBRATION_PODS_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <bag_of_tricks/high_res_timer.h>
#include <bag_of_tricks/image_region_iterator.h>
#include <pipeline/pod.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <xpl_calibration/descriptor_database.h>
#include <xpl_calibration/organized_connected_components.h>
#include <xpl_calibration/pcd_stream.h>
//typedef op::PCDStream<pcl::PointXYZRGB> Stream;
typedef rgbd::StreamSequenceAccessor Stream;
typedef Stream::Ptr StreamPtr;
typedef Stream::ConstPtr StreamConstPtr;


#endif // XPL_CALIBRATION_PODS_H
