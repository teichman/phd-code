#include <xpl_calibration/slam_calibration_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

SlamCalibrationVisualizer::SlamCalibrationVisualizer(SlamCalibrator::Ptr calibrator) :
  calibrator_(calibrator),
  map_(new Cloud),
  quitting_(false),
  needs_update_(false)
{
  vis_.registerKeyboardCallback(&SlamCalibrationVisualizer::keyboardCallback, *this);
  vis_.setBackgroundColor(1, 1, 1);
  
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

void SlamCalibrationVisualizer::run()
{
  //boost::thread thread_slam(boost::bind(&SlamVisualizer::slamThreadFunction, this));
  // Apparently PCLVisualizer needs to run in the main thread.
  Cloud::Ptr map = calibrator_->buildMap(0);
  *map_ = *map;
  needs_update_ = true;
  cout << "Done building map." << endl;
  
  visualizationThreadFunction();

  //thread_slam.join();
}

void SlamCalibrationVisualizer::setCamera(const std::string& camera_path)
{
  int argc = 3;
  char* argv[argc];
  argv[0] = (char*)string("aoeu").c_str();
  argv[1] = (char*)string("-cam").c_str();
  argv[2] = (char*)camera_path.c_str();
  bool success = vis_.getCameraParameters(argc, argv);
  ROS_ASSERT(success);
  vis_.updateCamera();    
}

void SlamCalibrationVisualizer::visualizationThreadFunction()
{
  while(scopeLockRead, !quitting_) {
    usleep(5e3);
    
    lockWrite();
    if(needs_update_)
      if(!vis_.updatePointCloud(map_, "default"))
	vis_.addPointCloud(map_, "default");
    vis_.spinOnce(3);
    needs_update_ = false;
    unlockWrite();
  }
}

void SlamCalibrationVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{

}


