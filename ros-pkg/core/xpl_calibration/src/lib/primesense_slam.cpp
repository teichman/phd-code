#include <xpl_calibration/primesense_slam.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

#define VISUALIZE

PrimeSenseSlam::PrimeSenseSlam() :
  fav_(NULL),
  min_dt_(0.2),
  max_range_(3.5),
  loopclosure_step_(5),
  keypoints_per_frame_(100)
{
}

void PrimeSenseSlam::_run()
{
  FrameAligner aligner(sseq_->model_, sseq_->model_, max_range_);
#ifdef VISUALIZE
  if(fav_)
    aligner.view_handler_ = fav_;
#endif

  ROS_WARN("PrimeSenseSlam does not use learned model.");
  pgs_ = PoseGraphSlam::Ptr(new PoseGraphSlam(sseq_->size()));
  Matrix6d covariance = Matrix6d::Identity() * 1e-3;  // TODO: Do something smarter with link covariances.

  Frame curr_frame, prev_frame;
  size_t prev_idx;
  sseq_->readFrame(0, &curr_frame);
  size_t curr_idx = 0;
  // Cache the features for the first frame.
  vector<cv::KeyPoint> unused;
  getFeatures(curr_frame, curr_idx, unused);
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
    
    // -- Compute orb features for that frame.
    vector<cv::KeyPoint> curr_keypoints;
    FeaturesPtr curr_features = getFeatures(curr_frame, curr_idx, curr_keypoints);
    
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
    			       curr_features, feature_cache_[prev_idx],
    			       true, &curr_to_prev);
    // bool found = aligner.align(curr_frame, prev_frame, &curr_to_prev);
    if(found) {
      cout << "Added edge " << prev_idx << " -- " << curr_idx << endl;
      pgs_->addEdge(prev_idx, curr_idx, curr_to_prev, covariance);
    }

    // -- Try to find loop closure links to some previous frames.
    for(size_t i = 0; i < (size_t)max<int>(0, (int)cached_frames_.size() - 2); i += loopclosure_step_) {
      size_t idx = cached_frames_[i];
      ROS_DEBUG_STREAM("Checking for loop closure between " << idx << " and " << curr_idx);
      Frame old_frame;
      sseq_->readFrame(idx, &old_frame);

      ROS_ASSERT(keypoint_cache_.count(idx));
      ROS_ASSERT(feature_cache_.count(idx));
      Affine3d curr_to_old;
      if(fav_)
    	fav_->setFrames(curr_frame, old_frame);
      bool found = aligner.align(curr_frame, old_frame,
      				 curr_keypoints, keypoint_cache_[idx],
      				 curr_features, feature_cache_[idx],
      				 false, &curr_to_old);
      //bool found = aligner.align(curr_frame, old_frame, &curr_to_old);
      if(found) {
    	cout << "Added edge " << idx << " -- " << curr_idx << endl;
    	pgs_->addEdge(idx, curr_idx, curr_to_old, covariance);
      }
    }
  }

  // -- Run slam solver and get final trajectory and pcd.
  pgs_->solve();
  traj_.resize(pgs_->numNodes());
  for(size_t i = 0; i < pgs_->numNodes(); ++i) {
    if(pgs_->numEdges(i) == 0)
      continue;
    traj_.set(i, pgs_->transform(i));
  }
  buildMap(traj_);

  // -- Clean up.
  quitting_ = true;
  if(fav_)
    fav_->quit();
}

// TODO: This seems to come up a lot and should be made more generally available.
void PrimeSenseSlam::buildMap(const Trajectory& traj)
{
  pcl::VoxelGrid<rgbd::Point> vg;
  vg.setLeafSize(0.02, 0.02, 0.02);
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
    vg.setInputCloud(map_);
    vg.filter(*tmp);
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

