#include <xpl_calibration/slam_calibration_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

SlamCalibrationVisualizer::SlamCalibrationVisualizer(SlamCalibrator::Ptr calibrator) :
  calibrator_(calibrator),
  map_(new Cloud),
  quitting_(false),
  needs_update_(false),
  seq_idx_(0),
  frame_idx_(0)
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

  setSequenceIdx(0);
  incrementFrameIdx(1);
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
  while(true) {
    { scopeLockRead; if(quitting_) break; }
    usleep(5e3);
    
    lockWrite();
    if(needs_update_) {
      cout << "updating" << endl;
      Cloud::Ptr pcd(new Cloud);
      *pcd = *map_;
      if(!vis_.updatePointCloud(pcd, "default"))
	vis_.addPointCloud(pcd, "default");
      needs_update_ = false;
    }

    vis_.removeCoordinateSystem();
    vis_.addCoordinateSystem(0.3, calibrator_->trajectories_[seq_idx_].get(frame_idx_).cast<float>());
    
    unlockWrite();
    vis_.spinOnce(3);
  }
}

void SlamCalibrationVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.keyDown()) {
    char key = event.getKeyCode();
    if(key == 27) {
      scopeLockWrite;
      quitting_ = true;
    }
    else if(key == '>')
      incrementSequenceIdx(1);
    else if(key == '<')
      incrementSequenceIdx(-1);
    else if(key == '.')
      incrementFrameIdx(1);
    else if(key == ',')
      incrementFrameIdx(-1);
  }
}


void SlamCalibrationVisualizer::setSequenceIdx(size_t idx)
{
  Cloud::Ptr map = calibrator_->buildMap(idx, 0.03);
  cout << "Done building map." << endl;
  cout << map->size() << " points." << endl;

  scopeLockWrite;
  seq_idx_ = idx;
  map_ = map;
  needs_update_ = true;
}

void SlamCalibrationVisualizer::incrementSequenceIdx(int num)
{
  int idx = (int)seq_idx_ + num;
  if(idx < 0)
    idx = 0;
  if((size_t)idx > calibrator_->size())
    idx = calibrator_->size();

  setSequenceIdx((size_t)idx);
}

void SlamCalibrationVisualizer::incrementFrameIdx(int num)
{
  int idx = (int)frame_idx_;
  int incr = 1;
  if(num < 0)
    incr = -1;

  const Trajectory& traj = calibrator_->trajectories_[seq_idx_];
  while(num != 0) {
    // Find the next valid transform.
    while(true) {
      idx += incr;
      if(idx < 0)
	idx = traj.size() - 1;
      if(idx >= (int)traj.size())
	idx = 0;

      if(traj.exists(idx)) {
	break;
      }
    }
    num -= incr;
  }
  
  scopeLockWrite;
  frame_idx_ = (size_t)idx;
}
