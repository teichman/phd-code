#ifndef VIS_WRAPPER_H
#define VIS_WRAPPER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <bag_of_tricks/lockable.h>
#include <rgbd_sequence/rgbd_sequence.h>

namespace rgbd
{

  class VisWrapper : public Lockable
  {
  public:
    pcl::visualization::PCLVisualizer vis_;

    VisWrapper(std::string name = "VisWrapper", double coordinate_system_size = 0.1);
    //! This may have to be called from the main thread.
    char waitKey(int msec = 0);
    void showCloud(rgbd::Cloud::ConstPtr pcd);
  
  protected:
    char key_;
    std::string name_;
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  };

}

#endif // VIS_WRAPPER_H
