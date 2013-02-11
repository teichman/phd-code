#ifndef BOX_SELECTION_VISUALIZER_H
#define BOX_SELECTION_VISUALIZER_H

#include <bag_of_tricks/agent.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <xpl_calibration/slam_calibrator.h>

class BoxSelectionVisualizer : public Agent
{
public:
  BoxSelectionVisualizer(rgbd::Cloud::Ptr pcd, std::string output_path);
  ~BoxSelectionVisualizer() {}

  void _run();
  
protected:
  pcl::visualization::PCLVisualizer vis_;
  rgbd::Cloud::Ptr pcd_;
  rgbd::Cloud::Ptr pcdvis_;
  rgbd::Point center_;
  double width_x_;
  double width_y_;
  double width_z_;
  bool needs_update_;
  vector<bool> inside_;
  std::string output_path_;

  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
  void drawVisualization();
  void save();
};

#endif // BOX_SELECTION_VISUALIZER_H
