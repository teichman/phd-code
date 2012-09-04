#include <xpl_calibration/slam_visualizer.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

SlamVisualizer::SlamVisualizer()
{
  vis_.registerKeyboardCallback(&SlamVisualizer::keyboardCallback, *this);
  vis_.addCoordinateSystem(1.0);
  
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

void SlamVisualizer::run(StreamSequence::ConstPtr sseq)
{
  sseq_ = sseq;
  
  boost::thread thread_slam(boost::bind(&SlamVisualizer::slamThreadFunction, this));
  // Apparently PCLVisualizer needs to run in the main thread.
  visualizationThreadFunction();
  thread_slam.join();
}

void SlamVisualizer::slamThreadFunction()
{
  FrameAligner aligner(sseq_->model(), sseq_->model());
  PoseGraphSlam slam(sseq_->size());
  
  Frame prev_frame;
  Frame curr_frame;
  for(size_t i = 1; i < sseq_->size(); ++i) { 
    // -- Add the next link.
    sseq_->readFrame(i-1, &prev_frame);
    sseq_->readFrame(i, &curr_frame);
    Affine3f curr_to_prev = aligner.align(prev_frame, curr_frame);
    Matrix3d rotation = 
    slam->addEdge(i, i-1, translation, rotation, covariance);

    // -- Solve.

    // -- Update the display.
    updateVisualizationCloud();
  }
}

void SlamVisualizer::updateVisualizationCloud()
{
  // -- Build the visualization cloud.
  Cloud::Ptr pcd(new Cloud);
  
  // -- Update the PCLVisualizer.
  lockWrite();
  if(!vis_.updatePointCloud(pcd, "default"))
    vis_.addPointCloud(pcd, "default");
  unlockWrite();
}

void SlamVisualizer::visualizationThreadFunction()
{
  while(true)
    vis_.spinOnce(1);
}

void SlamVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.keyDown()) {
    cout << "Pressed " << (int)key_ << endl;
  }
}
