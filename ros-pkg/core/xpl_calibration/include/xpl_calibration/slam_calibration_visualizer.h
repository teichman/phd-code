#ifndef SLAM_CALIBRATION_VISUALIZER_H
#define SLAM_CALIBRATION_VISUALIZER_H

#include <boost/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <bag_of_tricks/lockable.h>
#include <rgbd_sequence/stream_sequence.h>
#include <xpl_calibration/trajectory.h>
#include <xpl_calibration/slam_calibrator.h>

class SlamCalibrationVisualizer : public SharedLockable
{
public:
  SlamCalibrationVisualizer(SlamCalibrator::Ptr calibrator);
  void run();
  void setCamera(const std::string& camera_path);
  
protected:
  pcl::visualization::PCLVisualizer vis_;
  SlamCalibrator::Ptr calibrator_;
  rgbd::Cloud::Ptr map_;
  bool quitting_;
  bool needs_update_;
  size_t seq_idx_;
  size_t frame_idx_;

  void visualizationThreadFunction();
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  void setSequenceIdx(size_t idx);
  void incrementSequenceIdx(int num);
  void incrementFrameIdx(int num);
};

#endif // SLAM_CALIBRATION_VISUALIZER_H
