#ifndef SLAM_VISUALIZER_H
#define SLAM_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <bag_of_tricks/lockable.h>
#include <pose_graph_slam/pose_graph_slam.h>
#include <rgbd_sequence/stream_sequence.h>
#include <xpl_calibration/frame_aligner.h>

class SlamVisualizer : public SharedLockable
{
public:
  SlamVisualizer();
  void run(rgbd::StreamSequence::ConstPtr sseq);
  
protected:
  pcl::visualization::PCLVisualizer vis_;
  rgbd::StreamSequence::ConstPtr sseq_;

  void slamThreadFunction();
  void visualizationThreadFunction();
  //! Makes a new aggegate Cloud::Ptr and sends it to the visualizer.
  void updateVisualizationCloud();
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
};

#endif // SLAM_VISUALIZER_H
