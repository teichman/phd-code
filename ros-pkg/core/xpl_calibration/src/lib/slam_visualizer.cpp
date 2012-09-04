#include <xpl_calibration/slam_visualizer.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

SlamVisualizer::SlamVisualizer() :
  map_(new Cloud),
  curr_pcd_(new Cloud),
  curr_pcd_transformed_(new Cloud),
  incr_(30),
  needs_update_(false)
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
  *map_ = *sseq_->getCloud(0);
  lockWrite();
  needs_update_ = true;
  unlockWrite();
  
  FrameAligner aligner(sseq_->model_, sseq_->model_, this);
  slam_ = PoseGraphSlam::Ptr(new PoseGraphSlam(sseq_->size()));
  Matrix6d covariance = Matrix6d::Identity() * 1e-3;  
  
  Frame prev_frame;
  Frame curr_frame;
  for(size_t i = incr_; i < sseq_->size(); i += incr_) {
    cout << "---------- Searching for link between " << i - incr_ << " and " << i << endl;
    lockWrite();
    curr_pcd_ = sseq_->getCloud(i);
    unlockWrite();
    
    // -- Add the next link.
    sseq_->readFrame(i-incr_, &prev_frame);
    sseq_->readFrame(i, &curr_frame);
    Affine3d curr_to_prev;
    if(getenv("NO_ALIGNMENT"))
      curr_to_prev = Affine3d::Identity();
    else
      curr_to_prev = aligner.align(curr_frame, prev_frame);
    slam_->addEdge(i, i-incr_, curr_to_prev, covariance);

    // -- Solve.
    slam_->solve();

    // -- Update the map.
    lockWrite();
    for(size_t j = 0; j < slam_->numNodes(); ++j) {
      Affine3f transform = slam_->transform(j).cast<float>();
      cout << "*** transform " << j << ": " << endl << transform.matrix() << endl;
    }
    Affine3f transform = slam_->transform(i).cast<float>();
    cout << "Final transform: " << endl << transform.matrix() << endl;
    Cloud::Ptr pcdi = sseq_->getCloud(i);
    pcl::transformPointCloud(*pcdi, *pcdi, transform);
    *map_ += *pcdi;
    curr_pcd_->clear();
    curr_pcd_transformed_->clear();
    needs_update_ = true;
    unlockWrite();
  }
}

void SlamVisualizer::visualizationThreadFunction()
{
  while(true) {
    lockWrite();
    if(needs_update_) {
      Cloud::Ptr vis(new Cloud);
      *vis = *map_;
      *vis += *curr_pcd_transformed_;
      if(!vis_.updatePointCloud(vis, "default"))
	vis_.addPointCloud(vis, "default");
    }
    vis_.spinOnce(1);
    if(needs_update_) {
      static int num = 0;
      ostringstream oss;
      oss << "slam" << setw(5) << setfill('0') << num << ".png";
      vis_.saveScreenshot(oss.str());
      ++num;
    }
    needs_update_ = false;
    unlockWrite();
  }
}

void SlamVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.keyDown()) {
    cout << "Pressed " << event.getKeyCode() << endl;
  }
}

void SlamVisualizer::handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective)
{
  cout << "Improvement: objective " << objective << " at " << x.transpose() << endl;
  Affine3f transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));

  lockWrite();
  pcl::transformPointCloud(*curr_pcd_, *curr_pcd_transformed_, transform);
  needs_update_ = true;
  unlockWrite();
}

