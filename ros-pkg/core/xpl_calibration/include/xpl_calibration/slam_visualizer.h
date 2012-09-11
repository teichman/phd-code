#ifndef SLAM_VISUALIZER_H
#define SLAM_VISUALIZER_H

#include <boost/thread.hpp>
#include <gperftools/profiler.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <bag_of_tricks/lockable.h>
#include <pose_graph_slam/pose_graph_slam.h>
#include <rgbd_sequence/stream_sequence.h>
#include <xpl_calibration/frame_aligner.h>
#include <xpl_calibration/loop_closer.h>

class SlamVisualizer : public SharedLockable, public GridSearchViewHandler
{
public:
  SlamVisualizer();
  void run(rgbd::StreamSequence::ConstPtr sseq);
  
protected:
  pcl::visualization::PCLVisualizer vis_;
  rgbd::StreamSequence::ConstPtr sseq_;
  PoseGraphSlam::Ptr slam_;
  rgbd::Cloud::Ptr map_;
  rgbd::Cloud::Ptr curr_pcd_;
  rgbd::Cloud::Ptr curr_pcd_transformed_;
  size_t incr_;
  bool needs_update_;
  bool save_imgs_;
  Eigen::Affine3d tip_transform_;
  bool quitting_;
  pcl::VoxelGrid<rgbd::Point> vg_;
  
  void slamThreadFunction();
  void visualizationThreadFunction();
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  void handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective);
  //SDM added: loop closure
  LoopCloser::Ptr lc_;
};

#endif // SLAM_VISUALIZER_H
