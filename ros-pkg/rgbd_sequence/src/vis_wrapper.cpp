#include <rgbd_sequence/vis_wrapper.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::visualization;
using namespace rgbd;

namespace rgbd { 

  VisWrapper::VisWrapper(std::string name) :
    Lockable(),
    key_(0),
    name_(name)
  {
    vis_.registerKeyboardCallback(&VisWrapper::keyboardCallback, *this);
    vis_.addCoordinateSystem();
    
    // -- Set the viewpoint to be sensible for PrimeSense devices.
    vis_.camera_.clip[0] = 0.00387244;
    vis_.camera_.clip[1] = 3.87244;
    vis_.camera_.focal[0] = -0.160878;
    vis_.camera_.focal[1] = -0.0444743;
    vis_.camera_.focal[2] = 1.281;
    vis_.camera_.pos[0] = 0.0402195;
    vis_.camera_.pos[1] = 0.0111186;
    vis_.camera_.pos[2] = -1.7;
    vis_.camera_.view[0] = 0;
    vis_.camera_.view[1] = -1;
    vis_.camera_.view[2] = 0;
    vis_.camera_.window_size[0] = 1678;
    vis_.camera_.window_size[1] = 525;
    vis_.camera_.window_pos[0] = 2;
    vis_.camera_.window_pos[1] = 82;
    vis_.updateCamera();    
  }
  
  char VisWrapper::waitKey(int msec)
  {
    lock();
    
    key_ = 0;
    if(msec == 0)
      while(key_ == 0)
	vis_.spinOnce(1);
    else
      vis_.spinOnce(msec);

    unlock();
    return key_;
  }
  
  void VisWrapper::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
  {
    if(event.keyDown()) {
      key_ = event.getKeyCode();
      // cout << "Pressed " << (int)key_ << endl;
    }
  }

  void VisWrapper::showCloud(rgbd::Cloud::ConstPtr pcd)
  {
    lock();

    if(!vis_.updatePointCloud(pcd, "default"))
      vis_.addPointCloud(pcd, "default");

    unlock();
  }

} // namespace rgbd
