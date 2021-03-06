#ifndef LINK_VISUALIZER_H
#define LINK_VISUALIZER_H

#include <boost/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <agent/lockable.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <xpl_calibration/trajectory.h>
#include <xpl_calibration/utility_functions.h>
#include <pcl/common/transforms.h>
#include <xpl_calibration/primesense_slam.h>
#include <string>

class LinkVisualizer : public SharedLockable
{
public:
  LinkVisualizer(rgbd::StreamSequenceBase::ConstPtr sseq, PoseGraphSlam::Ptr slam);
  void run();
  void setCamera(const std::string& camera_path);
protected:
  enum ViewMode
  {
    PREV,
    CUR,
    ALL
  };
  pcl::visualization::PCLVisualizer vis_;
  rgbd::StreamSequenceBase::ConstPtr sseq_;
  PoseGraphSlam::Ptr slam_;
  size_t edge_idx_;
  const EdgeStruct* cur_edge_;
  bool quitting_;
  bool transform_;
  bool needs_update_;
  bool sorted_;
  std::string frame_text_;
  std::vector<double> errors_;
  std::vector<double> errors_desc_;
  std::vector<size_t> edges_desc_;
  ViewMode view_mode_;
  
  void visualizationThreadFunction();
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  void incrementEdgeIdx(int amt);
  void toggleTransform();
  void toggleSort();
  void setViewMode(ViewMode mode);
};

#endif //LINK_VISUALIZER_H
