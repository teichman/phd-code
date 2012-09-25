#include <xpl_calibration/frame_aligner.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

FrameAligner::FrameAligner(const rgbd::PrimeSenseModel& model0,
			   const rgbd::PrimeSenseModel& model1,
			   double max_range,
			   GridSearchViewHandler* view_handler) :
  num_ransac_samples_(1000),
  k_(5),
  max_feature_dist_(300),
  min_ransac_inliers_(5),
  min_pairwise_keypoint_dist_(0.04),
  ransac_max_inlier_dist_(0.02),
  min_ransac_inlier_percent_(0.1),
  min_bounding_length_(.04),  
  max_range_(max_range),
  model0_(model0),
  model1_(model1),
  view_handler_(view_handler)
{
}

bool FrameAligner::align(rgbd::Frame frame0, rgbd::Frame frame1, Eigen::Affine3d* f0_to_f1) const
{
  vector<cv::Point2d> correspondences0, correspondences1;
  return wideGridSearch(frame0, frame1, correspondences0, correspondences1, f0_to_f1);
}

bool FrameAligner::align(rgbd::Frame frame0, rgbd::Frame frame1,
			 const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
			 FeaturesConstPtr features0, FeaturesConstPtr features1,
			 bool consider_wide_search, Eigen::Affine3d* f0_to_f1) const
{
  // -- Try to get rough transform between the two based on the corresponding keypoints.
  Affine3d guess;
  vector<cv::Point2d> correspondences0, correspondences1;
  bool found_rough_transform = computeRoughTransform(frame0, frame1, keypoints0, keypoints1, features0, features1, &correspondences0, &correspondences1, &guess);
  
  // -- Run grid search as desired.
  if(found_rough_transform)
    return narrowGridSearch(frame0, frame1, correspondences0, correspondences1, guess, f0_to_f1);
  else if(consider_wide_search)
    return wideGridSearch(frame0, frame1, correspondences0, correspondences1, f0_to_f1);
  else
    return false;
}

bool FrameAligner::wideGridSearch(rgbd::Frame frame0, rgbd::Frame frame1,
				      const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1,
				      Eigen::Affine3d* f0_to_f1) const
{
  ROS_DEBUG("FrameAligner::wideGridSearch");
  // -- Run grid search.
  ScopedTimer st("FrameAligner::align");
  FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(model0_, model1_, frame0, frame1, correspondences0, correspondences1, max_range_, 0.25));
  GridSearch gs(6);
  gs.verbose_ = false;
  gs.view_handler_ = view_handler_;
  gs.objective_ = mde;
  gs.num_scalings_ = 5;
  double max_res_rot = 1.5 * M_PI / 180.0;
  double max_res_trans = 0.05;
  gs.max_resolutions_ << max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 2;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  ArrayXd x = gs.search(ArrayXd::Zero(6));

  // -- Print out statistics.
  cout << "============================== Frame alignment complete" << endl;
  cout << "GridSearch solution: " << x.transpose() << endl;
  cout << "Computed " << gs.num_evals_ << " evals in " << gs.time_ << " seconds." << endl;
  cout << gs.num_evals_ / gs.time_ << " evals / second." << endl;
  cout << gs.time_ / gs.num_evals_ << " seconds / eval." << endl;

  double count;
  mde->count_ = &count;
  double final_objective = mde->eval(x);
  
  cout << " -- Single-number statistics" << endl;
  cout << "Final objective: " << final_objective << endl;
  cout << "Count: " << count << endl;
  cout << "==============================" << endl;

  if(validate(count, final_objective)) {
    *f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
    return true;
  }
  else
    return false;
}

bool FrameAligner::validate(double count, double final_objective) const
{
  if(count < 20000 || final_objective > 0.5) {
    ROS_WARN("Alignment finished but was not considered successful..");
    return false;
  }
  
  // if(final_objective > 0.04) {
  //   ROS_WARN("Alignment finished but was not considered successful..");
  //   return false;
  // }
  
  return true;
}

bool FrameAligner::narrowGridSearch(rgbd::Frame frame0, rgbd::Frame frame1,
				    const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1, 
				    const Eigen::Affine3d& guess,
				    Eigen::Affine3d* f0_to_f1) const
{
  ROS_DEBUG("FrameAligner::narrowGridSearch");
  // -- Run grid search.
  ScopedTimer st("FrameAligner::align");
  FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(model0_, model1_, frame0, frame1, correspondences0, correspondences1, max_range_, 0.25));
  GridSearch gs(6);
  gs.verbose_ = false;
  gs.view_handler_ = view_handler_;
  gs.objective_ = mde;
  gs.num_scalings_ = 5;
  double max_res_rot = 1.5 * M_PI / 180.0;
  double max_res_trans = 0.02;
  gs.max_resolutions_ << max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 1;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  //Convert guess to XYZRPY
  double rx, ry, rz, tx, ty, tz;
  generateXYZYPR(guess.cast<float>(), rx, ry, rz, tx, ty, tz);
  //Now search with that initialization
  Eigen::ArrayXd init(6); init << rx, ry, rz, tx, ty, tz;
  Eigen::ArrayXd x = gs.search(init);

  // -- Print out statistics.
  cout << "============================== Frame alignment complete" << endl;
  cout << "GridSearch solution: " << x.transpose() << endl;
  cout << "Computed " << gs.num_evals_ << " evals in " << gs.time_ << " seconds." << endl;
  cout << gs.num_evals_ / gs.time_ << " evals / second." << endl;
  cout << gs.time_ / gs.num_evals_ << " seconds / eval." << endl;

  double count;
  mde->count_ = &count;
  double final_objective = mde->eval(x);
  
  cout << " -- Single-number statistics" << endl;
  cout << "Final objective: " << final_objective << endl;
  cout << "Count: " << count << endl;
  cout << "==============================" << endl;

  if(validate(count, final_objective)) {
    *f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
    return true;
  }
  else
    return false;
}

bool FrameAligner::computeRoughTransform(rgbd::Frame frame0, rgbd::Frame frame1,
					 const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
					 FeaturesConstPtr features0, FeaturesConstPtr features1,
					 std::vector<cv::Point2d>* correspondences0, std::vector<cv::Point2d>* correspondences1,
					 Eigen::Affine3d* f0_to_f1) const
{
  correspondences0->clear();
  correspondences1->clear();
  
  cv::FlannBasedMatcher matcher;
  vector< vector<cv::DMatch> > matches_each;
  // TODO: Why was this inside a try / catch?
  matcher.knnMatch(*features0, *features1, matches_each, k_);

  // -- Get best correspondences for use in grid search and build up flat vector of all matches
  //    that meet our criteria.
  vector<cv::DMatch> matches;
  correspondences0->reserve(matches_each.size());
  correspondences1->reserve(matches_each.size());
  for(size_t j = 0; j < matches_each.size(); j++) {
    const vector<cv::DMatch>& mat = matches_each[j];
    if(mat.empty())
      continue;

    bool added_corr = false;
    for(size_t k = 0; k < mat.size(); k++) {
      //cout << mat[k].distance << " ";
      if(mat[k].queryIdx < 0 || mat[k].trainIdx < 0)
	continue;
      if(mat[k].distance > max_feature_dist_)
	continue;

      // Make sure opencv is doing what we think it is.
      if(k > 0)
	ROS_ASSERT(mat[k].distance >= mat[k-1].distance);
      
      // Correspondences get filled with the best matches that are at least somewhat good.
      // They don't need depth.
      if(!added_corr) {
	correspondences0->push_back(keypoints0[mat[k].queryIdx].pt);
	correspondences1->push_back(keypoints1[mat[k].trainIdx].pt);
	added_corr = true;
      }

      // Saved matches must have depth for 3d ransac.
      if(frame0.depth_->coeffRef(keypoints0[mat[k].queryIdx].pt.y, keypoints0[mat[k].queryIdx].pt.x) > 0 &&
	 frame1.depth_->coeffRef(keypoints1[mat[k].trainIdx].pt.y, keypoints1[mat[k].trainIdx].pt.x) > 0)
      {
	matches.push_back(mat[k]);
      }
    }
  }

  //Check that enough are matched
  if(matches.size() < min_ransac_inliers_)
    return false;

  //Visualize
  // if(visualize_)
  // {
  //   Frame old_frame; sseq_->readFrame(old_t, &old_frame);
  //   cv::Mat3b img_match;
  //   cv::drawMatches(frame.img_, keypoints0, old_frame.img_, keypoints1,
  // 		    matches, img_match);
  //   cv::imshow("match", img_match);
  //   cv::waitKey(50);
  // }

  // TODO: Don't need to project the whole frame here, just keypoints.
  Cloud::Ptr cloud0(new Cloud);
  Cloud::Ptr cloud1(new Cloud);
  model0_.frameToCloud(frame0, cloud0.get());
  model1_.frameToCloud(frame1, cloud1.get());

  //From here the logic is more or less copied from orb_matcher
  vector<Eigen::Affine3f> candidates;
  for(int j = 0; j < num_ransac_samples_; ++j)
  {
    //Sample 3 points in ref img and precompute their distances to each other
    const cv::DMatch &match0 = matches[rand() % matches.size()];
    const cv::DMatch &match1 = matches[rand() % matches.size()];
    const cv::DMatch &match2 = matches[rand() % matches.size()];
    int idx0 = match0.queryIdx;
    int idx1 = match1.queryIdx;
    int idx2 = match2.queryIdx;
    pcl::PointXYZRGB r0 = cloud0->at(keypoints0[idx0].pt.x, keypoints0[idx0].pt.y);
    pcl::PointXYZRGB r1 = cloud0->at(keypoints0[idx1].pt.x, keypoints0[idx1].pt.y);
    pcl::PointXYZRGB r2 = cloud0->at(keypoints0[idx2].pt.x, keypoints0[idx2].pt.y);
    ROS_ASSERT(isFinite(r0) && isFinite(r1) && isFinite(r2));
    double d01 = pcl::euclideanDistance(r0, r1);
    double d02 = pcl::euclideanDistance(r0, r2);
    double d12 = pcl::euclideanDistance(r1, r2);
    if(d01 < min_pairwise_keypoint_dist_)
      continue;
    if(d02 < min_pairwise_keypoint_dist_)
      continue;
    if(d12 < min_pairwise_keypoint_dist_)
      continue;
    //Get their corresponding matches
    int oidx0 = match0.trainIdx;
    int oidx1 = match1.trainIdx;
    int oidx2 = match2.trainIdx;
    pcl::PointXYZRGB t0 = cloud1->at(keypoints1[oidx0].pt.x, keypoints1[oidx0].pt.y);
    pcl::PointXYZRGB t1 = cloud1->at(keypoints1[oidx1].pt.x, keypoints1[oidx1].pt.y);
    pcl::PointXYZRGB t2 = cloud1->at(keypoints1[oidx2].pt.x, keypoints1[oidx2].pt.y);
    ROS_ASSERT(isFinite(t0) && isFinite(t1) && isFinite(t2));
    //If the matches are too distant from themselves w.r.t the distance in the current frame, continue
    if(fabs(pcl::euclideanDistance(t0, t1) - d01) > ransac_max_inlier_dist_)
      continue;
    if(fabs(pcl::euclideanDistance(t0, t2) - d02) > ransac_max_inlier_dist_)
      continue;
    if(fabs(pcl::euclideanDistance(t1, t2) - d12) > ransac_max_inlier_dist_)
      continue;
    //Otherwise generate transformation from correspondence
    pcl::TransformationFromCorrespondences tfc;
    tfc.add(t0.getVector3fMap(), r0.getVector3fMap());
    tfc.add(t1.getVector3fMap(), r1.getVector3fMap());
    tfc.add(t2.getVector3fMap(), r2.getVector3fMap());
    
    Eigen::Affine3f trans = tfc.getTransformation();
    if((trans * t0.getVector3fMap() - r0.getVector3fMap()).norm() > 0.01 ||
       (trans * t1.getVector3fMap() - r1.getVector3fMap()).norm() > 0.01 ||
       (trans * t2.getVector3fMap() - r2.getVector3fMap()).norm() > 0.01)
      continue;
    candidates.push_back(trans);
  }
  cout << "Have " << candidates.size() << " candidates after RANSAC, before inlier check" << endl;
  //Do inlier check
  Cloud::Ptr keypoint_cloud0(new Cloud);
  for(size_t j = 0; j < keypoints0.size(); ++j) {
    Point pt = cloud0->at(keypoints0[j].pt.x, keypoints0[j].pt.y);
    if(isFinite(pt))
      keypoint_cloud0->push_back(pt);
  }
  Cloud::Ptr keypoint_cloud1(new Cloud);
  for(size_t j = 0; j < keypoints1.size(); ++j) {
    Point pt = cloud1->at(keypoints1[j].pt.x, keypoints1[j].pt.y);
    if(isFinite(pt))
      keypoint_cloud1->push_back(pt);
  }
  
  pcl::KdTreeFLANN<Point> kdtree0;
  kdtree0.setInputCloud(keypoint_cloud0);
  
  Cloud transformed_keypoint_cloud1;
  vector<int> indices;
  vector<float> distances;
  Eigen::Affine3f best_transform;
  float best_distance = std::numeric_limits<float>::infinity();
  size_t best_num_inliers = 0;
  bool has_best = false;
  vector< std::pair<size_t, size_t> > best_inliers;
  for(size_t j = 0; j < candidates.size(); ++j)
  {
    float inlier_distance = 0;
    //Ensure that, in at least 2 directions, there's variance here
    float minx=std::numeric_limits<float>::infinity();
    float maxx=-std::numeric_limits<float>::infinity();
    float miny=std::numeric_limits<float>::infinity();
    float maxy=-std::numeric_limits<float>::infinity();
    float minz=std::numeric_limits<float>::infinity();
    float maxz=-std::numeric_limits<float>::infinity();
    pcl::transformPointCloud(*keypoint_cloud1, transformed_keypoint_cloud1, candidates[j]);
    int num_inliers = 0;
    vector< std::pair<size_t, size_t> > inliers;
    for(size_t k = 0; k < transformed_keypoint_cloud1.size(); ++k)
    {
      indices.clear();
      distances.clear();
      kdtree0.nearestKSearch(transformed_keypoint_cloud1.points[k], 1, indices, distances);
      if(distances.size() != 0 && distances[0] < ransac_max_inlier_dist_)
      {
	++num_inliers;
	inlier_distance += distances[0];
	inliers.push_back(std::pair<size_t, size_t>(k, indices[0]));
	const pcl::PointXYZRGB& kpt = transformed_keypoint_cloud1.points[k];
	//Update bounding volume
	if(kpt.x < minx) minx = kpt.x;
	if(kpt.x > maxx) maxx = kpt.x;
	if(kpt.y < miny) miny = kpt.y;
	if(kpt.y > maxy) maxy = kpt.y;
	if(kpt.z < minz) minz = kpt.z;
	if(kpt.z > maxz) maxz = kpt.z;
      }
    }
    //Check number of inliers
    if(num_inliers < min_ransac_inlier_percent_ * keypoint_cloud0->size() || num_inliers < min_ransac_inliers_)
    {
      continue;
    }
    //Check bounding volume
    int has_x = (maxx - minx) > min_bounding_length_;
    int has_y = (maxy - miny) > min_bounding_length_;
    int has_z = (maxz - minz) > min_bounding_length_;
    if(has_x + has_y + has_z < 2)
    {
      continue;
    }
    
    // Consider this transform
    inlier_distance /= num_inliers; //Average out
    if(inlier_distance < best_distance)
    {
      best_transform = candidates[j];
      best_distance = inlier_distance;
      best_num_inliers = num_inliers;
      best_inliers = inliers;
      has_best = true;
    }
  }
  if(has_best)
  {
    // Re-compute the transform using only the inliers.
    pcl::TransformationFromCorrespondences tfc;
    for(size_t j = 0; j < best_inliers.size(); j++)
      tfc.add(keypoint_cloud1->points[best_inliers[j].first].getVector3fMap(), 
	      keypoint_cloud0->points[best_inliers[j].second].getVector3fMap());
    Affine3d f1_to_f0 = tfc.getTransformation().cast<double>();
    *f0_to_f1 = f1_to_f0.inverse();
    return true;
  }
  else
    return false;
}


FrameAlignmentVisualizer::FrameAlignmentVisualizer(rgbd::Cloud::Ptr cloud0, rgbd::Cloud::Ptr cloud1) :
						   
  cloud0_(cloud0),
  cloud1_(cloud1),
  needs_update_(false)
{
}

void FrameAlignmentVisualizer::handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective) const
{
  cout << "Grid search improvement: " << objective << " at x = " << x.transpose() << endl;
  scopeLockWrite;
  f0_to_f1_ = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
  needs_update_ = true;  
}

void FrameAlignmentVisualizer::_run()
{
  while(scopeLockRead, !quitting_) {
    usleep(1e3);
    lockWrite();
    if(needs_update_) {
      Cloud::Ptr pcd(new Cloud);
      pcl::transformPointCloud(*cloud0_, *pcd, f0_to_f1_);
      *pcd += *cloud1_;
      if(!vis_.updatePointCloud(pcd, "default"))
	vis_.addPointCloud(pcd, "default");
    }
    vis_.spinOnce(3);
    unlockWrite();
  }
}
