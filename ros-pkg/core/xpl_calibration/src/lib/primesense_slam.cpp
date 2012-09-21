#include <xpl_calibration/primesense_slam.h>

PrimeSenseSlam::PrimeSenseSlam() :
  min_dt_(0.2),
  max_range_(3.5)
{
}

void PrimeSenseSlam::_run()
{
  FrameAligner aligner(model, model, max_range_);
  ROS_WARN("PrimeSenseSlam does not use learned model.");
  slam_ = PoseGraphSlam::Ptr(new PoseGraphSlam(sseq_->size()));
  Matrix6d covariance = Matrix6d::Identity() * 1e-3;  

  size_t prev_idx;
  sseq_->readFrame(0, &curr_frame_);
  size_t curr_idx = 0;
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
    }
    if(done) break;
    cout << "---------- Searching for link between " << prev_idx << " and " << curr_idx
	 << " / " << sseq_->size() << endl;
    cout << "           dt: " << dt << endl;
    sseq_->readFrame(prev_idx, &prev_frame_);  // TODO: This should not be necessary.
    sseq_->readFrame(curr_idx, &curr_frame_);
    
    // -- Compute orb features for that frame.
    vector<cv::KeyPoint> curr_keypoints;
    FeaturesPtr curr_features = getFeatures(curr_frame_, curr_idx, curr_keypoints);

    // -- Get corresponding keypoints between curr_frame_ and prev_frame_.
    ROS_ASSERT(keypoint_cache_.count(prev_idx));
    ROS_ASSERT(feature_cache_.count(prev_idx));
    std::vector<cv::Point2d> curr_keypoints_corr, prev_keypoints_corr;
    computeFeatureCorrespondences(curr_keypoints, curr_features,
				  keypoint_cache_[prev_idx], feature_cache_[prev_idx],
				  &curr_keypoints_corr, &prev_keypoints_corr);
    
    // -- Try to find link to most recent previous frame.
    //    Tries a wider search if not enough corresponding points to get a rough initial transform.
    Affine3d curr_to_prev;
    bool found = aligner.align(curr_frame_, prev_frame_, curr_keypoints_corr, prev_keypoints_corr, true, &curr_to_prev);
    if(found) {
      cout << "Added edge " << prev_idx << " -- " << curr_idx << endl;
      slam_->addEdge(prev_idx, curr_idx, curr_to_prev, covariance);
    }

    // -- Try to find loop closure links to all other previous frames.
    for(size_t i = 0; i < cached_frames_.size(); ++i) {
      size_t idx = cached_frames_[i];
      Frame old_frame;
      sseq_->readFrame(idx, &old_frame);

      ROS_ASSERT(keypoint_cache_.count(idx));
      ROS_ASSERT(feature_cache_.count(idx));
      std::vector<cv::Point2d> old_keypoints_corr;
      computeFeatureCorrespondences(curr_keypoints, curr_features,
				    keypoint_cache_[idx], feature_cache_[idx],
				    &curr_keypoints_corr, &old_keypoints_corr);

      Affine3d curr_to_old;
      bool found = aligner.align(curr_frame_, old_frame, curr_keypoints_corr, old_keypoints_corr, false, &curr_to_old);
      if(found) {
	cout << "Added edge " << idx << " -- " << curr_idx << endl;
	slam_->addEdge(idx, curr_idx, curr_to_old, covariance);
      }
    }
  }

  // -- Run slam solver and get final trajectory and pcd.
  slam_->solve();
  traj_.resize(slam_->numNodes());
  for(size_t i = 0; i < slam_->numNodes(); ++i) {
    if(slam_->numEdges(i) == 0)
      continue;
    traj_.set(i, slam_->transform(i));
  }
  buildMap(traj_);

  // -- Clean up.
  quitting_ = true;
}

void PrimeSenseSlam::buildMap(const Trajectory& traj)
{
  map_ = Cloud::Ptr(new Cloud);
  for(size_t i = 0; i < traj.size(); i++) {
    if(!traj.exists(i))
      continue;
    
    Cloud::Ptr curr_pcd = sseq_->getCloud(i);
    zthresh(curr_pcd, max_range_);
    Cloud::Ptr curr_pcd_transformed(new Cloud);
    pcl::transformPointCloud(*curr_pcd, *curr_pcd_transformed, traj.get(i).cast<float>());
    *map_ += *curr_pcd_transformed;

    Cloud::Ptr tmp(new Cloud);
    vg_.setInputCloud(map_);
    vg_.filter(*tmp);
    *map_ = *tmp;
  }
}

PrimeSenseSlam::FeaturesPtr PrimeSenseSlam::getFeatures(const rgbd::Frame &frame, 
							size_t t, vector<cv::KeyPoint> &keypoints)
{
  cv::Mat1b img;
  cv::cvtColor(frame.img_, img, CV_BGR2GRAY);
  cv::Mat1b mask = cv::Mat1b::zeros(img.rows, img.cols);
  mask = 255;  // Use everything.
  cv::ORB orb(keypoints_per_frame_);
  vector<cv::KeyPoint> keypoints;
  cv::Mat cvfeat;
  orb(img, mask, keypoints, cvfeat);

  FeaturesPtr features(new cv::Mat1f(keypoints.size(), cvfeat.cols));
  for(size_t i = 0; i < keypoints.size(); i++)
    for(int j  = 0; j < cvfeat.cols; j++)
      (*features)(i,j) = cvfeat.at<uint8_t>(i,j);
  
  keypoint_cache_[t] = keypoints;
  feature_cache_[t] = features;
  cached_frames_.push_back(t);
  return features;
}

void PrimeSenseSlam::computeFeatureCorrespondences(const std::vector<cv::KeyPoint>& curr_keypoints, FeaturesPtr curr_features,
						   const std::vector<cv::KeyPoint>& prev_keypoints, FeaturesPtr prev_features,
						   const std::vector<cv::Point2d>* curr_keypoints_corr,
						   const std::vector<cv::Point2d>* prev_keypoints_corr) const
{
  
}
