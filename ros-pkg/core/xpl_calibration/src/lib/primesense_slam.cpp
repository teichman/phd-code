#include <xpl_calibration/primesense_slam.h>

#define OMP_THREADS 8

using namespace std;
using namespace Eigen;
using namespace rgbd;

//#define VISUALIZE

PrimeSenseSlam::PrimeSenseSlam() :
  fav_(NULL),
  intrinsics_(NULL),
  params_(defaultParams())
{
  min_dt_ = params_.get<double>("min_dt");
  max_loopclosures_ = params_.get<int>("max_loopclosures");
  max_loopclosure_tests_ = params_.get<int>("max_loopclosure_tests");
  keypoints_per_frame_ = params_.get<int>("keypoints_per_frame");
}

void PrimeSenseSlam::_run()
{
  cout << "PrimseSenseSlam is using params: " << endl;
  cout << params_ << endl;
  FrameAligner aligner(sseq_->model_, sseq_->model_);
  cout << "FrameAligner is using params: " << endl;
  cout << aligner.params_ << endl;
#ifdef VISUALIZE
  if(fav_)
    aligner.view_handler_ = fav_;
#endif

  pgs_ = PoseGraphSlam::Ptr(new PoseGraphSlam(sseq_->size()));
  Matrix6d covariance = Matrix6d::Identity() * 1e-3;  // TODO: Do something smarter with link covariances.

  Frame curr_frame, prev_frame;
  size_t prev_idx;
  sseq_->readFrame(0, &curr_frame);
  if(intrinsics_)
    intrinsics_->undistort(&curr_frame);
  size_t curr_idx = 0;
  vector<cv::KeyPoint> unused;
  rgbd::Cloud::ConstPtr unused2;
  cacheFeatures(curr_frame, curr_idx, unused, unused2);
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
    }
    if(done) break;
    cout << "---------- Searching for link between " << prev_idx << " and " << curr_idx
	 << " / " << sseq_->size() << endl;
    cout << "           dt: " << dt << endl;
    sseq_->readFrame(prev_idx, &prev_frame);  // TODO: This should not be necessary.      
    sseq_->readFrame(curr_idx, &curr_frame);
    if(intrinsics_) {
      intrinsics_->undistort(&prev_frame);
      intrinsics_->undistort(&curr_frame);
    }
    
    // -- Compute orb features for that frame.
    vector<cv::KeyPoint> curr_keypoints;
    rgbd::Cloud::ConstPtr curr_keycloud;
    FeaturesPtr curr_features = cacheFeatures(curr_frame, curr_idx, curr_keypoints, 
					      curr_keycloud);
    
    // -- Try to find link to most recent previous frame.
    //    Tries a wider search if not enough corresponding points to get a rough initial transform.
    ROS_DEBUG_STREAM("Computing frame alignment between " << prev_idx << " and " << curr_idx);
    ROS_ASSERT(keypoint_cache_.count(prev_idx));
    ROS_ASSERT(feature_cache_.count(prev_idx));
    Affine3d curr_to_prev;
    if(fav_)
      fav_->setFrames(curr_frame, prev_frame);
    bool found = aligner.align(curr_frame, prev_frame,
    			       curr_keypoints, keypoint_cache_[prev_idx],
			       curr_keycloud, keycloud_cache_[prev_idx],
    			       curr_features, feature_cache_[prev_idx],
    			       true, &curr_to_prev);
    if(found) {
      cout << "Added edge " << prev_idx << " -- " << curr_idx << endl;
      pgs_->addEdge(prev_idx, curr_idx, curr_to_prev, covariance);
    }

    // -- Try to find loop closure links to some previous frames.
    if(max_loopclosures_ > 0) { 
      vector<size_t> cached_frames_random;
      ROS_ASSERT(!cached_frames_.empty() && cached_frames_.back() == curr_idx);
      if(cached_frames_.size() > 1)
	ROS_ASSERT(cached_frames_[cached_frames_.size() - 2] == prev_idx);
      cached_frames_random.insert(cached_frames_random.end(), cached_frames_.begin(), cached_frames_.end() - 2);
      if(!cached_frames_random.empty()) {
	ROS_ASSERT(cached_frames_random.back() != curr_idx);
	ROS_ASSERT(cached_frames_random.back() != prev_idx);
      }
      random_shuffle(cached_frames_random.begin(), cached_frames_random.end());
      size_t num_rough_transforms = 0;
      size_t max_rough_transforms = max_loopclosures_ * 2; //HEURISTIC, see if it holds
      // BEGIN parallelism
      size_t max_attempts = cached_frames_random.size() < max_loopclosure_tests_ ? 
        cached_frames_random.size() : max_loopclosure_tests_;
      vector<bool> found_rough(max_attempts, false);
      vector<vector<cv::Point2d> > correspondences0(max_attempts), correspondences1(max_attempts);
      vector<Eigen::Affine3d> guesses(max_attempts);
      omp_set_num_threads(OMP_THREADS);
#pragma omp parallel for
      for(size_t i = 0; i < max_attempts; ++i) {
	if(num_rough_transforms >= max_rough_transforms)
	  continue;
	size_t idx = cached_frames_random[i];
	ROS_DEBUG_STREAM("Checking for loop closure between " << idx << " and " << curr_idx);

	ROS_ASSERT(keypoint_cache_.count(idx));
	ROS_ASSERT(feature_cache_.count(idx));
	bool found = aligner.computeRoughTransform(
	  curr_keypoints, keypoint_cache_[idx],
	  curr_keycloud, keycloud_cache_[idx],
	  curr_features, feature_cache_[idx],
	  &(correspondences0[i]), &(correspondences1[i]), &(guesses[i]));
	if(found) {
	  found_rough[i] = true;
#pragma omp critical
	  {++num_rough_transforms;}
	}
      }
      size_t num_successful_loopclosures = 0;
      for(size_t i = 0; i < found_rough.size(); i++){
	if(!found_rough[i]) continue;
	size_t idx = cached_frames_random[i];
	ROS_DEBUG_STREAM("Densely aligning loop closure between " << idx << " and " << curr_idx);
	Frame old_frame;
	sseq_->readFrame(idx, &old_frame);
	if(intrinsics_)
	  intrinsics_->undistort(&old_frame);

	if(fav_)
	  fav_->setFrames(curr_frame, old_frame);
	Eigen::Affine3d curr_to_old;
	bool found = aligner.narrowGridSearch(
	  curr_frame, old_frame,
	  correspondences0[i], correspondences1[i], guesses[i], &curr_to_old);
	if(found) {
	  cout << "Added edge " << idx << " -- " << curr_idx << endl;
	  pgs_->addEdge(idx, curr_idx, curr_to_old, covariance);
	  ++num_successful_loopclosures;
	  if(num_successful_loopclosures >= max_loopclosures_)
	    break;
	}
      }
    }
  }
  populateTrajAndMaps();
  // -- Clean up.
  quitting_ = true;
  if(fav_)
    fav_->quit();  
}

void PrimeSenseSlam::populateTrajAndMaps()
{
  // -- Run slam solver and get final trajectory and pcd.
  pgs_->solve(params_.get<int>("min_subgraph_size"));
  vector<vector<int> > subgraphs; pgs_->getSubgraphs(subgraphs); 
  trajs_.resize(subgraphs.size());
  maps_.resize(subgraphs.size());
  for(size_t i = 0; i < subgraphs.size(); i++)
  {
    Trajectory &traj = trajs_[i];
    const vector<int> &subgraph = subgraphs[i];
    traj.resize(pgs_->numNodes());
    for(size_t j = 0; j < subgraph.size(); ++j)
      traj.set(subgraph[j], pgs_->transform(subgraph[j]));
    maps_[i] = buildMap(traj);
  }
}

// TODO: This seems to come up a lot and should be made more generally available.
Cloud::Ptr PrimeSenseSlam::buildMap(const Trajectory& traj) const
{
  pcl::VoxelGrid<rgbd::Point> vg;
  vg.setLeafSize(0.02, 0.02, 0.02);
  Cloud::Ptr map(new Cloud);
  int num_frames_used = 0;
  for(size_t i = 0; i < traj.size(); i++) {
    if(!traj.exists(i))
      continue;
    cout << "Using frame " << i << " / " << traj.size() << endl;
    Cloud::Ptr curr_pcd = sseq_->getCloud(i);
    Cloud::Ptr nonans(new Cloud);
    nonans->reserve(curr_pcd->size());
    for(size_t j = 0; j < curr_pcd->size(); ++j)
      if(isFinite(curr_pcd->at(j)))
	nonans->push_back(curr_pcd->at(j));
    zthresh(nonans, MAX_RANGE_MAP);
    Cloud::Ptr curr_pcd_transformed(new Cloud);
    pcl::transformPointCloud(*nonans, *curr_pcd_transformed, traj.get(i).cast<float>());
    *map += *curr_pcd_transformed;
    
    num_frames_used++;
    if(num_frames_used % 250 == 0)
    {
      cout << "Filtering" << endl;
      Cloud::Ptr tmp(new Cloud);
      vg.setInputCloud(map);
      vg.filter(*tmp);
      *map = *tmp;
    }
  }
  cout << "Filtering" << endl;
  Cloud::Ptr tmp(new Cloud);
  vg.setInputCloud(map);
  vg.filter(*tmp);
  *map = *tmp;
  return map;
}

PrimeSenseSlam::FeaturesPtr PrimeSenseSlam::cacheFeatures(const rgbd::Frame &frame, 
							  size_t t, vector<cv::KeyPoint> &keypoints, rgbd::Cloud::ConstPtr &keycloud)
{
  FeaturesPtr features = getFeatures(frame, keypoints, keycloud);
  keypoint_cache_[t] = keypoints;
  keycloud_cache_[t] = keycloud;
  feature_cache_[t] = features;
  cached_frames_.push_back(t);
  return features;
}

PrimeSenseSlam::FeaturesPtr PrimeSenseSlam::getFeatures(const rgbd::Frame &frame, vector<cv::KeyPoint> &keypoints, rgbd::Cloud::ConstPtr &keycloud) const
{
  cv::Mat1b img;
  cv::cvtColor(frame.img_, img, CV_BGR2GRAY);
  cv::Mat1b mask = cv::Mat1b::zeros(img.rows, img.cols);
  mask = 255;  // Use everything.
  cv::ORB orb(keypoints_per_frame_);
  cv::Mat cvfeat;
  orb(img, mask, keypoints, cvfeat);

  FeaturesPtr features(new cv::Mat1f(keypoints.size(), cvfeat.cols));
  for(size_t i = 0; i < keypoints.size(); i++)
    for(int j  = 0; j < cvfeat.cols; j++)
      (*features)(i,j) = cvfeat.at<uint8_t>(i,j);
  
  // Make keycloud
  rgbd::Cloud* keycloudptr = new rgbd::Cloud;
  keycloudptr->points.resize(keypoints.size());
  keycloudptr->height = 1;
  keycloudptr->width = keypoints.size();
  for(size_t i = 0; i < keypoints.size(); i++)
  {
    const cv::KeyPoint &kpt = keypoints[i];
    rgbd::ProjectivePoint kppt;
    kppt.u_ = kpt.pt.x;
    kppt.v_ = kpt.pt.y;
    kppt.z_ = frame.depth_->coeffRef(kppt.v_, kppt.u_);
    sseq_->model_.project(kppt, &(keycloudptr->points[i]));
  }
  keycloud = rgbd::Cloud::ConstPtr((const rgbd::Cloud*) keycloudptr);
  
  return features;
}

