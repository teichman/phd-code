#include <xpl_calibration/slam_visualizer.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

#define GRIDSEARCH_VIS (getenv("GRIDSEARCH_VIS") ? bool(atoi(getenv("GRIDSEARCH_VIS"))) : false)
#define MAX_FRAMES (getenv("MAX_FRAMES") ? atoi(getenv("MAX_FRAMES")) : 0)

SlamVisualizer::SlamVisualizer() :
  max_range_(3.5),
  min_dt_(0.2),
  save_imgs_(false),
  map_(new Cloud),
  curr_pcd_(new Cloud),
  curr_pcd_transformed_(new Cloud),
  incr_(5),
  needs_update_(false),
  needs_map_rebuild_(false),
  tip_transform_(Affine3d::Identity()),
  use_loop_closure_(false),
  quitting_(false),
  frame_text_(""),
  vis_("PCLVisualizer")
{
  vis_.registerKeyboardCallback(&SlamVisualizer::keyboardCallback, *this);
  vis_.setBackgroundColor(1, 1, 1);
  //vis_.addCoordinateSystem(1.0);
  vg_.setLeafSize(0.02, 0.02, 0.02);
  
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

void SlamVisualizer::setCamera(const std::string& camera_path)
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

void SlamVisualizer::run(StreamSequence::ConstPtr sseq,
			 const std::string& opcd_path,
			 const std::string& otraj_path)
{
  sseq_ = sseq;
  // Initialize loop closure
  if(use_loop_closure_)
  {
    lc_ = LoopCloser::Ptr(new LoopCloser(sseq));
    lc_->fine_tune_ = false; //Doing this by hand for visualization
    lc_->visualize_ = false;
    lc_->min_time_offset_ = 30;
    lc_->step_ = 3;
    lc_->max_feature_dist_ = 500;
    lc_->keypoints_per_frame_ = 250;
    lc_->min_pairwise_keypoint_dist_ = 0.1; //cm apart
    lc_->min_ransac_inliers_ = 10; //Need at least this many inliers to be considered a valid
    lc_->min_ransac_inlier_percent_ = 0.1;
    lc_->ransac_max_inlier_dist_ = 0.02; //Need to be within 2 cm to be considered an inlier
    lc_->num_ransac_samples_ = 1000;
    lc_->icp_max_avg_dist_ = 0.03; //Avg pt-to-pt distance required
    lc_->icp_max_inlier_dist_ = 0.1; // Highest distance allowed to be considered an inlier
    lc_->icp_inlier_percent_ = 0.3; // At least this percentage of points must be inliers
    lc_->ftype_ = ORB;
    lc_->k_ = 2;
    lc_->verification_type_ = MDE;
    lc_->max_mde_ = 0.1;
    lc_->harris_thresh_ = 0.01;
    lc_->use_3d_sift_ = true;
    lc_->fpfh_radius_ = 0.02;
    lc_->harris_margin_ = 50;
    lc_->view_handler_ = (GridSearchViewHandler*) this;
  }
  opcd_path_ = opcd_path;
  otraj_path_ = otraj_path;
  
  boost::thread thread_slam(boost::bind(&SlamVisualizer::slamThreadFunction, this));
  // Apparently PCLVisualizer needs to run in the main thread.
  visualizationThreadFunction();
  thread_slam.join();
}

void SlamVisualizer::slamThreadFunction()
{
  lockWrite();
  *map_ = *sseq_->getCloud(0);
  zthresh(map_, max_range_);
  needs_update_ = true;
  unlockWrite();

  PrimeSenseModel model = sseq_->model_;
  ROS_WARN("SlamVisualizer does not use learned model.");
  FrameAligner aligner(model, model, max_range_, this);
  slam_ = PoseGraphSlam::Ptr(new PoseGraphSlam(sseq_->size()));
  Matrix6d covariance = Matrix6d::Identity() * 1e-3;  
  
  nodes_.clear();
  size_t prev_idx;
  sseq_->readFrame(0, &curr_frame_);
  size_t curr_idx = 0;
  nodes_.push_back(curr_idx);
  while(true) {
    // -- Find the next frame to use.
    prev_frame_ = curr_frame_;
    prev_idx = curr_idx;
    double dt = 0;
    bool done = false;
    while(dt < min_dt_) {
      ++curr_idx;
      if(curr_idx >= sseq_->size()) {
	done = true;
	break;
      }
      dt = sseq_->timestamps_[curr_idx] - prev_frame_.timestamp_;
      //cout << "Searching: " << curr_idx << " " << prev_idx << " " << dt << endl;
    }
    if(done) break;
    //lockWrite();
    ostringstream oss;
    oss << "ODOMETRY: Aligning frame " << curr_idx << " -> " << prev_idx;
    frame_text_ = oss.str();
    //unlockWrite();
    cout << "---------- Searching for link between " << prev_idx << " and " << curr_idx
	 << " / " << sseq_->size() << endl;
    cout << "           dt: " << dt << endl;
    sseq_->readFrame(prev_idx, &prev_frame_);
    sseq_->readFrame(curr_idx, &curr_frame_);
    cv::imshow("current", curr_frame_.depthImage());
    cv::imshow("previous", prev_frame_.depthImage());
    cv::waitKey(5);

    // -- Load the cloud for visualization purposes.
    lockWrite();
    curr_pcd_ = sseq_->getCloud(curr_idx);
    zthresh(curr_pcd_, max_range_);
    if(quitting_) {
      unlockWrite();
      break;
    }
    unlockWrite();
    
    // -- Add the next link.
    if(getenv("PROFILE"))
      ProfilerStart("slam_test.prof");
    double count, final_mde;
    vector<cv::Point2d> curr_keypoints, prev_keypoints;
    // for(int i = 0; i < 2000; ++i) {
    //   cv::Point2d pt;
    //   pt.x = rand() % 640;
    //   pt.y = rand() % 480;
    //   curr_keypoints.push_back(pt);
    //   prev_keypoints.push_back(pt);
    // }
    Affine3d curr_to_prev = aligner.align(curr_frame_, prev_frame_, curr_keypoints, prev_keypoints, &count, &final_mde);
    if(getenv("PROFILE"))
      ProfilerStop();

    // -- For now, terminate at the first broken link.  With loop closure we can do better.
    // mde of 0.2 for depth-only seems good. //COUNT WAS 20000
    if(count < 20000 || final_mde > 0.5) {
      cout << "Edge has count " << count << " and final_mde " << final_mde << ".  Terminating." << endl;
      break;
    }
    else
      slam_->addEdge(prev_idx, curr_idx, curr_to_prev, covariance);
    //Loop closure
    vector<size_t> targets;
    vector<Eigen::Affine3f> transforms;
    lockWrite();
    oss.clear();
    oss.str("");
    oss << "LOOP CLOSURE: Aligning frame " << curr_idx << " -> " << "?";
    frame_text_ = oss.str();
    unlockWrite();
    if(use_loop_closure_ && lc_->getLinkHypotheses(curr_frame_, curr_idx, targets, transforms))
    {
      for(size_t j = 0; j < targets.size(); j++)
      {
        if(!lc_->fine_tune_)
        {
          lockWrite();
          oss.clear();
          oss.str("");
          oss << "LOOP CLOSURE: Aligning frame " << curr_idx << " -> " << targets[j];
          frame_text_ = oss.str();
          tip_transform_ = slam_->transform(targets[j]); //Must update tip
          needs_update_ = true;
          unlockWrite();
          rgbd::Frame frame_target;
          sseq_->readFrame(targets[j], &frame_target);
          transforms[j] = lc_->fineTuneHypothesis(curr_frame_, frame_target, transforms[j]);
        }
        cout << "Adding loop edge with transform: " << endl << transforms[j].matrix() << endl;
        slam_->addEdge(targets[j], curr_idx, transforms[j].cast<double>(), covariance);
      }
      if(GRIDSEARCH_VIS)
        needs_map_rebuild_ = true; //Only bother rebuilding the map if we're visualizing gridsearch
    }
    
    // -- Solve.  For now this isn't really doing anything other than showing
    //    that we have the input and output transforms correct.
    slam_->solve();
    nodes_.push_back(curr_idx);

    // -- Update the map.
    lockWrite();
    Affine3d transform = slam_->transform(curr_idx);
    Cloud::Ptr pcdi = sseq_->getCloud(curr_idx);
    zthresh(pcdi, max_range_);
    pcl::transformPointCloud(*pcdi, *pcdi, transform.cast<float>());
    *map_ += *pcdi;
    curr_pcd_->clear();
    curr_pcd_transformed_->clear();
    tip_transform_ = transform;
    needs_update_ = true;
    unlockWrite();

    // -- Check for early termination.
    if(MAX_FRAMES != 0 && curr_idx > MAX_FRAMES)
      break;
  }
  rebuild_map();
  // -- Save the output and shut down.
  usleep(1e6);  // Let the visualizer filter the map.
  scopeLockWrite;
  if(opcd_path_ != "") {
    pcl::io::savePCDFileBinary(opcd_path_, *map_);
    cout << "Saved final map to " << opcd_path_ << endl;
  }

  if(otraj_path_ != "") { 
    Trajectory traj;
    traj.resize(slam_->numNodes());
    for(size_t i = 0; i < slam_->numNodes(); ++i) {
      if(slam_->numEdges(i) == 0)
	continue;
      traj.set(i, slam_->transform(i));
    }
    traj.save(otraj_path_);
    cout << "Saved trajectory to " << otraj_path_ << endl;
  }

  quitting_ = true;
}
  
void SlamVisualizer::rebuild_map()
{
  //lockWrite();
  cout << "Rebuilding the map!" << endl;
  *map_ = Cloud();
  for(size_t i = 0; i < nodes_.size(); i++)
  {
    //Get the index
    size_t idx = nodes_[i];
    //Get the cloud
    Cloud::Ptr curr_pcd = sseq_->getCloud(idx);
    zthresh(curr_pcd, max_range_);
    Cloud::Ptr curr_pcd_transformed(new Cloud);
    //Transform it
    Eigen::Affine3d trans = slam_->transform(idx);
    pcl::transformPointCloud(*curr_pcd, *curr_pcd_transformed, trans.cast<float>());
    *map_ += *curr_pcd_transformed;
    //Filter it
    Cloud::Ptr vis(new Cloud);
    vg_.setInputCloud(map_);
    vg_.filter(*vis);
    *map_ = *vis;
    if(i == nodes_.size() - 1)
      tip_transform_ = trans;
  }
  //unlockWrite();
}

void SlamVisualizer::visualizationThreadFunction()
{
  while(scopeLockRead, !quitting_) {
//    ScopedTimer st("SlamVisualizer::visualizationThreadFunction loop");
    usleep(5e3);
    
    lockWrite();
    if(needs_update_) {
      Cloud::Ptr vis(new Cloud);
      if(needs_map_rebuild_)
      {
        rebuild_map();
        needs_map_rebuild_ = false;
        *vis = *map_;
      }
      else
      {
        vg_.setInputCloud(map_);
        vg_.filter(*vis);
        *map_ = *vis;
      }

        *vis += *curr_pcd_transformed_;
      if(!vis_.updatePointCloud(vis, "default"))
	vis_.addPointCloud(vis, "default");
      int xpos = 20; int ypos = 10; int fontsize = 25;
      string toshow = frame_text_;
      if(!GRIDSEARCH_VIS) toshow += " (WARNING: MAP NOT BEING REBUILT. MAY BE SIGNIFICANTLY OFF IN VIS)";
      if(!vis_.updateText(toshow, xpos, ypos, fontsize, 0, 0, 0, "label"))
  vis_.addText(frame_text_, xpos, ypos, fontsize, 0, 0, 0, "label");
    }

    vis_.spinOnce(3);

    if(needs_update_ && save_imgs_) {
      static int num = 0;

      // -- Save just the alignment output.
      ostringstream oss;
      oss << "slam" << setw(5) << setfill('0') << num << ".png";
      vis_.saveScreenshot(oss.str());

      // -- Get depth images too.
      cv::Mat3b slam = cv::imread(oss.str(), 1);
      cv::Mat3b prev_depthimage = prev_frame_.depthImage();
      cv::Mat3b curr_depthimage = curr_frame_.depthImage();

      // -- Scale down the images.
      double scale = slam.rows / ((double)prev_depthimage.rows * 2);
      cv::Size sz;
      sz.width = prev_depthimage.cols * scale;
      sz.height = prev_depthimage.rows * scale;
      cv::Mat3b prev_depthimage_scaled;
      cv::resize(prev_depthimage, prev_depthimage_scaled, sz);
      cv::Mat3b curr_depthimage_scaled;
      cv::resize(curr_depthimage, curr_depthimage_scaled, sz);
      ROS_ASSERT(prev_depthimage_scaled.rows + curr_depthimage_scaled.rows == slam.rows);
      ROS_ASSERT(prev_depthimage_scaled.cols == curr_depthimage_scaled.cols);

      // -- Assemble the montage.
      cv::Mat3b montage(slam.rows, slam.cols + prev_depthimage_scaled.cols);
      for(int y = 0; y < slam.rows; ++y)
	for(int x = 0; x < slam.cols; ++x)
	  montage(y, x) = slam(y, x);
      for(int y = 0; y < prev_depthimage_scaled.rows; ++y)
	for(int x = 0; x < prev_depthimage_scaled.cols; ++x)
	  montage(y, x + slam.cols) = prev_depthimage_scaled(y, x);
      for(int y = 0; y < curr_depthimage_scaled.rows; ++y)
	for(int x = 0; x < curr_depthimage_scaled.cols; ++x)
	  montage(y + prev_depthimage_scaled.rows, x + slam.cols) = curr_depthimage_scaled(y, x);

      // -- Save.
      oss.clear();
      oss.str("");
      oss << "montage" << setw(5) << setfill('0') << num << ".png";
      cv::imwrite(oss.str(), montage);

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
  cout << "Message = " << frame_text_ << endl;

  if(GRIDSEARCH_VIS) {
    Affine3d transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
    lockWrite();
    pcl::transformPointCloud(*curr_pcd_, *curr_pcd_transformed_, (tip_transform_ * transform).cast<float>());
    needs_update_ = true;
    unlockWrite();
  }
}

