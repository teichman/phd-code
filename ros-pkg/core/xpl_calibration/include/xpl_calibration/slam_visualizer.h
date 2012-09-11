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
  //! Used in generating the map.
  double max_range_;
  //! When choosing the next frame, advance by at least this much.
  double min_dt_;
  //! If true, after each grid search improvement save a screenshot.
  bool save_imgs_;
  //! Use loop closure
  bool use_loop_closure_;
  //! Camera pose to use.
  // TODO
  
  SlamVisualizer();
  void run(rgbd::StreamSequence::ConstPtr sseq, const std::string& opcd_path);
  void setCamera(const std::string& camera_path);
  
protected:
  pcl::visualization::PCLVisualizer vis_;
  rgbd::StreamSequence::ConstPtr sseq_;
  PoseGraphSlam::Ptr slam_;
  rgbd::Cloud::Ptr map_;
  rgbd::Cloud::Ptr curr_pcd_;
  rgbd::Cloud::Ptr curr_pcd_transformed_;
  size_t incr_;
  bool needs_update_;
  Eigen::Affine3d tip_transform_;
  bool quitting_;
  pcl::VoxelGrid<rgbd::Point> vg_;
  std::string opcd_path_;
  
  void slamThreadFunction();
  void visualizationThreadFunction();
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  void handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective);
  //SDM added: loop closure
  LoopCloser::Ptr lc_;
};

#endif // SLAM_VISUALIZER_H
