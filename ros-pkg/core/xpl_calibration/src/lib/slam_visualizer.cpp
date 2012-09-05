#include <xpl_calibration/slam_visualizer.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

SlamVisualizer::SlamVisualizer() :
  map_(new Cloud),
  curr_pcd_(new Cloud),
  curr_pcd_transformed_(new Cloud),
  incr_(1),
  needs_update_(false),
  save_imgs_(false),
  tip_transform_(Affine3d::Identity()),
  quitting_(false)
{
  vis_.registerKeyboardCallback(&SlamVisualizer::keyboardCallback, *this);
  //vis_.addCoordinateSystem(1.0);
  vg_.setLeafSize(0.01, 0.01, 0.01);
  
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
  lockWrite();
  *map_ = *sseq_->getCloud(0);
  needs_update_ = true;
  unlockWrite();

  PrimeSenseModel model = sseq_->model_;
  model.use_distortion_model_ = false;
  FrameAligner aligner(model, model, this);
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
    ProfilerStart("slam_test.prof");
    Affine3d curr_to_prev = aligner.align(curr_frame, prev_frame);
    ProfilerStop();
    
    cout << "Adding edge with transform: " << endl << curr_to_prev.matrix() << endl;
    slam_->addEdge(i-incr_, i, curr_to_prev, covariance);
    cout << "Inverse of that: " << endl << curr_to_prev.inverse().matrix() << endl;
    
    // -- Solve.
    slam_->solve();

    // -- Update the map.
    lockWrite();
    Affine3d transform = slam_->transform(i);
    cout << "tip_transform_: " << endl << tip_transform_.matrix() << endl;
    cout << "Update from grid search: " << endl << curr_to_prev.matrix() << endl;
    cout << "Expected g2o result: " << endl << (curr_to_prev * tip_transform_).matrix() << endl;
    cout << "Final transform from g2o: " << endl << transform.matrix() << endl;
    Cloud::Ptr pcdi = sseq_->getCloud(i);
    pcl::transformPointCloud(*pcdi, *pcdi, transform.cast<float>());
    *map_ += *pcdi;
    curr_pcd_->clear();
    curr_pcd_transformed_->clear();
    tip_transform_ = transform;
    needs_update_ = true;
    unlockWrite();
  }
}

void SlamVisualizer::visualizationThreadFunction()
{
  while(scopeLockRead, !quitting_) {
//    ScopedTimer st("SlamVisualizer::visualizationThreadFunction loop");
    usleep(5e3);
    
    lockWrite();
    if(needs_update_) {
      Cloud::Ptr vis(new Cloud);
      vg_.setInputCloud(map_);
      vg_.filter(*vis);
      *map_ = *vis;

      *vis += *curr_pcd_transformed_;
      if(!vis_.updatePointCloud(vis, "default"))
	vis_.addPointCloud(vis, "default");
    }

    vis_.spinOnce(3);

    if(needs_update_ && save_imgs_) {
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
    cout << "Pressed " << (int)event.getKeyCode() << endl;

    if(event.getKeyCode() == 'd' || event.getKeyCode() == 27) {
      scopeLockWrite;
      quitting_ = true;
    }
    else if(event.getKeyCode() == 's') {
      scopeLockWrite;
      save_imgs_ = !save_imgs_;
    }
  }
}

void SlamVisualizer::handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective)
{
  //ScopedTimer st("SlamVisualizer::handleGridSearchUpdate");
  cout << "Improvement: objective " << objective << " at " << x.transpose() << endl;
  Affine3f transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));

  lockWrite();
  pcl::transformPointCloud(*curr_pcd_, *curr_pcd_transformed_, transform * tip_transform_.cast<float>());
  needs_update_ = true;
  unlockWrite();
}

