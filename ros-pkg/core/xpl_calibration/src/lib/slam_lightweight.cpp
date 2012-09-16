#include <xpl_calibration/slam_lightweight.h>

using namespace std;
using namespace g2o;
using namespace rgbd;

#define MAX_FRAMES (getenv("MAX_FRAMES") ? atoi(getenv("MAX_FRAMES")) : 0)

SlamLightweight::SlamLightweight() :
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
  frame_text_("")
{
  vg_.setLeafSize(0.02, 0.02, 0.02);
}

void SlamLightweight::run(StreamSequence::ConstPtr sseq,
			 const std::string& opcd_path,
			 const std::string& otraj_path,
       const std::string& ograph_path)
{
  sseq_ = sseq;
  // Initialize loop closure
  if(use_loop_closure_)
  {
    lc_ = LoopCloser::Ptr(new LoopCloser(sseq));
    lc_->fine_tune_ = false; //Doing this by hand for visualization
    lc_->visualize_ = false;
    lc_->min_time_offset_ = 12; // Was 18
    lc_->step_ = 2;  // Was 4
    lc_->max_feature_dist_ = 500;
    lc_->keypoints_per_frame_ = 250;
    lc_->min_pairwise_keypoint_dist_ = 0.1; //cm apart
    lc_->min_ransac_inliers_ = 10; //Need at least this many inliers to be considered a valid
    lc_->min_ransac_inlier_percent_ = 0.1;
    lc_->ransac_max_inlier_dist_ = 0.02; //Need to be within 2 cm to be considered an inlier
    lc_->num_ransac_samples_ = 1000;
    //lc_->icp_max_avg_dist_ = 0.03; //Avg pt-to-pt distance required
    //lc_->icp_max_inlier_dist_ = 0.1; // Highest distance allowed to be considered an inlier
    //lc_->icp_inlier_percent_ = 0.3; // At least this percentage of points must be inliers
    lc_->ftype_ = ORB;
    lc_->k_ = 2;
    lc_->verification_type_ = MDE;
    lc_->max_mde_ = 0.1;
    //lc_->harris_thresh_ = 0.01;
    //lc_->use_3d_sift_ = true;
    //lc_->fpfh_radius_ = 0.02;
    //lc_->harris_margin_ = 50;
    lc_->view_handler_ = (GridSearchViewHandler*) this;
  }
  opcd_path_ = opcd_path;
  otraj_path_ = otraj_path;
  ograph_path_ = ograph_path;
  slamThreadFunction();  
}

void SlamLightweight::slamThreadFunction()
{
  lockWrite();
  *map_ = *sseq_->getCloud(0);
  zthresh(map_, max_range_);
  needs_update_ = true;
  unlockWrite();

  PrimeSenseModel model = sseq_->model_;
  model.use_distortion_model_ = false;
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
    ostringstream oss;
    oss << "ODOMETRY: Aligning frame " << curr_idx << " -> " << prev_idx;
    frame_text_ = oss.str();
    cout << "---------- Searching for link between " << prev_idx << " and " << curr_idx
	 << " / " << sseq_->size() << endl;
    cout << "           dt: " << dt << endl;
    sseq_->readFrame(prev_idx, &prev_frame_);
    sseq_->readFrame(curr_idx, &curr_frame_);
    cv::imshow("current", curr_frame_.depthImage());
    cv::imshow("previous", prev_frame_.depthImage());
    cv::waitKey(5);

    // -- Add the next link.
    if(getenv("PROFILE"))
      ProfilerStart("slam_test.prof");
    double count, final_mde;
    Affine3d curr_to_prev = aligner.align(curr_frame_, prev_frame_, &count, &final_mde);
    if(getenv("PROFILE"))
      ProfilerStop();

    // -- For now, ignore at the first broken link.  With loop closure we can do better.
    // mde of 0.2 for depth-only seems good. //COUNT WAS 20000
    if(count < 20000 || final_mde > 0.5) {
      cout << "Edge has count " << count << " and final_mde " << final_mde << ".  Terminating." << endl;
      continue;
    }
    else
      slam_->addEdge(prev_idx, curr_idx, curr_to_prev, covariance);
    //Loop closure
    vector<size_t> targets;
    vector<Eigen::Affine3f> transforms;
    if(use_loop_closure_ && lc_->getLinkHypotheses(curr_frame_, curr_idx, targets, transforms))
    {
      for(size_t j = 0; j < targets.size(); j++)
      {
        if(!lc_->fine_tune_)
        {
          oss.clear();
          oss.str("");
          oss << "LOOP CLOSURE: Aligning frame " << curr_idx << " -> " << targets[j];
          frame_text_ = oss.str();
          rgbd::Frame frame_target;
          sseq_->readFrame(targets[j], &frame_target);
          transforms[j] = lc_->fineTuneHypothesis(curr_frame_, frame_target, transforms[j]);
        }
        cout << "Adding loop edge from " << curr_idx << " -> " << targets[j] << endl;
        slam_->addEdge(targets[j], curr_idx, transforms[j].cast<double>(), covariance);
      }
    }
    
    nodes_.push_back(curr_idx);

    // -- Check for early termination.
    if(MAX_FRAMES != 0 && curr_idx > MAX_FRAMES)
      break;
  }
  slam_->solve();
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

  if(ograph_path_ != "") {
    slam_->save(ograph_path_);
    cout << "Saved graph to " << ograph_path_ << endl;
  }

  quitting_ = true;
}
  
void SlamLightweight::rebuild_map()
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

void SlamLightweight::handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective)
{
  //ScopedTimer st("SlamLightweight::handleGridSearchUpdate");
  cout << "Improvement: objective " << objective << " at " << x.transpose() << endl;
  cout << "Message = " << frame_text_ << endl;
}

