#include <xpl_calibration/slam_visualizer.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

SlamVisualizer::SlamVisualizer() :
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
  // Initialize loop closure
  lc_ = LoopCloser::Ptr(new LoopCloser(sseq));
  lc_->fine_tune_ = true;
  lc_->visualize_ = false;
  lc_->max_orb_dist_ = 500;
  lc_->keypoints_per_frame_ = 500;
  lc_->min_keypoint_dist_ = 0.04; //cm apart
  lc_->min_inliers_ = 10; //Need at least this many inliers to be considered a valid
  lc_->min_inlier_percent_ = 0.1;
  lc_->distance_thresh_ = 0.03; //Need to be within 3 cm to be considered an inlier
  lc_->num_samples_ = 10000;
  lc_->icp_thresh_ = 0.02; //Avg pt-to-pt distance required
  lc_->icp_max_inlier_dist_ = 0.1; // Highest distance allowed to be considered an inlier
  lc_->icp_inlier_percent_ = 0.3; // At least this percentage of points must be inliers
  lc_->ftype_ = ORB;
  lc_->k_ = 5;
  lc_->min_time_offset_ = 15;
  lc_->verification_type_ = ICP;
  lc_->mde_thresh_ = 0.1;
  lc_->z_thresh_ = 3.0;
  
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
  Frame curr_frame;
  for(size_t i = incr_; i < sseq_->size(); i += incr_) {
    cout << "---------- Searching for link between " << i - incr_ << " and " << i << endl;
    lockWrite();
    curr_pcd_ = sseq_->getCloud(i);
    zthresh(curr_pcd_, 3);
    unlockWrite();
    
    // -- Add the next link.
    sseq_->readFrame(i-incr_, &prev_frame);
    sseq_->readFrame(i, &curr_frame);
    ProfilerStart("slam_test.prof");
    Affine3d curr_to_prev = aligner.align(curr_frame, prev_frame);
    ProfilerStop();
     
    cout << "Adding edge with transform: " << endl << curr_to_prev.matrix() << endl;
    slam_->addEdge(i-incr_, i, curr_to_prev, covariance);
    //Loop closure
    vector<size_t> targets;
    vector<Eigen::Affine3f> transforms;
    if(lc_->getLinkHypotheses(curr_frame, i, targets, transforms))
    {
      for(size_t j = 0; j < targets.size(); j++)
      {
        cout << "Adding loop edge with transform: " << endl << transforms[j].matrix() << endl;
        slam_->addEdge(targets[j], i, transforms[j].cast<double>(), covariance);
      }
    }
        
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

  Affine3f transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
  lockWrite();
  pcl::transformPointCloud(*curr_pcd_, *curr_pcd_transformed_, transform * tip_transform_.cast<float>());
  needs_update_ = true;
  unlockWrite();
}

