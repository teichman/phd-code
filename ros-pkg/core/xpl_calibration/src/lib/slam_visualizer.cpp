#include <xpl_calibration/slam_visualizer.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

SlamVisualizer::SlamVisualizer() :
  min_dt_(0.333),
  map_(new Cloud),
  curr_pcd_(new Cloud),
  curr_pcd_transformed_(new Cloud),
  incr_(5),
  needs_update_(false),
  save_imgs_(false),
  tip_transform_(Affine3d::Identity()),
  quitting_(false)
{
  vis_.registerKeyboardCallback(&SlamVisualizer::keyboardCallback, *this);
  //vis_.addCoordinateSystem(1.0);
  vg_.setLeafSize(0.01, 0.01, 0.01);
  //vis_.setBackgroundColor(1, 1, 1);
  
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
  zthresh(map_, 3);
  needs_update_ = true;
  unlockWrite();

  PrimeSenseModel model = sseq_->model_;
  model.use_distortion_model_ = false;
  FrameAligner aligner(model, model, this);
  slam_ = PoseGraphSlam::Ptr(new PoseGraphSlam(sseq_->size()));
  Matrix6d covariance = Matrix6d::Identity() * 1e-3;  
  
  Frame prev_frame;
  size_t prev_idx;
  Frame curr_frame;
  sseq_->readFrame(0, &curr_frame);
  size_t curr_idx = 0;
  while(true) {
    // -- Find the next frame to use.
    prev_frame = curr_frame;
    prev_idx = curr_idx;
    double dt = 0;
    bool done = false;
    while(dt < min_dt_) {
      ++curr_idx;
      if(curr_idx >= sseq_->size()) {
	done = true;
	break;
      }
      dt = sseq_->timestamps_[curr_idx] - prev_frame.timestamp_;
      //cout << "Searching: " << curr_idx << " " << prev_idx << " " << dt << endl;
    }
    if(done) break;
    cout << "---------- Searching for link between " << prev_idx << " and " << curr_idx
	 << " / " << sseq_->size() << endl;
    cout << "           dt: " << dt << endl;
    sseq_->readFrame(prev_idx, &prev_frame);
    sseq_->readFrame(curr_idx, &curr_frame);

    // -- Load the cloud for visualization purposes.
    lockWrite();
    curr_pcd_ = sseq_->getCloud(curr_idx);
    zthresh(curr_pcd_, 3);
    if(quitting_) {
      unlockWrite();
      break;
    }
    unlockWrite();
    
    // -- Add the next link.
    //ProfilerStart("slam_test.prof");
    double count, final_mde;
    Affine3d curr_to_prev = aligner.align(curr_frame, prev_frame, &count, &final_mde);
    //ProfilerStop();

    if(count < 20000 || final_mde > 0.1)
      cout << "Edge has count " << count << " and final_mde " << final_mde << ".  Rejecting." << endl;
    else
      slam_->addEdge(prev_idx, curr_idx, curr_to_prev, covariance);
    
    // -- Solve.  For now this isn't really doing anything other than showing
    //    that we have the input and output transforms correct.
    slam_->solve();

    // -- Update the map.
    lockWrite();
    Affine3d transform = slam_->transform(curr_idx);
    Cloud::Ptr pcdi = sseq_->getCloud(curr_idx);
    zthresh(pcdi, 3);
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

  Affine3d transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
  lockWrite();
  //pcl::transformPointCloud(*curr_pcd_, *curr_pcd_transformed_, (transform * tip_transform_).cast<float>());
  pcl::transformPointCloud(*curr_pcd_, *curr_pcd_transformed_, (tip_transform_ * transform).cast<float>());
  needs_update_ = true;
  unlockWrite();
}

