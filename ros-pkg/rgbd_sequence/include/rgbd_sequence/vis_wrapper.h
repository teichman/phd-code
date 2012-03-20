#ifndef VIS_WRAPPER_H
#define VIS_WRAPPER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <rgbd_sequence/rgbd_sequence.h>

namespace rgbd
{

  class VisWrapper
  {
  public:
    pcl::visualization::PCLVisualizer vis_;

    VisWrapper();
    char waitKey();
    void showCloud(rgbd::Cloud::ConstPtr pcd);
  
  protected:
    char key_;
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  };

}

#endif // VIS_WRAPPER_H
