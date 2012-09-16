/*
 * =====================================================================================
 *
 *       Filename:  loop_closer.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/05/2012 02:28:35 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */
#include <xpl_calibration/loop_closer.h>
#include <xpl_calibration/frame_aligner.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <xpl_calibration/object_matching_calibrator.h>
#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

using namespace rgbd;
using namespace std;

LoopCloser::LoopCloser(rgbd::StreamSequence::ConstPtr sseq):
  sseq_(sseq),
  fine_tune_(true),
  num_ransac_samples_(1000),
  min_ransac_inlier_percent_(0.1),
  min_time_offset_(15),
  min_pairwise_keypoint_dist_(0.04), //meters
  max_feature_dist_(500),
  ransac_max_inlier_dist_(0.02),
  k_(5),
  visualize_(false),
  keypoints_per_frame_(100),
  icp_max_avg_dist_(0.05),
  icp_max_inlier_dist_(0.04),
  icp_inlier_percent_(0.3),
  max_mde_(0.1),
  min_ransac_inliers_(5),
  min_bounding_length_(.04),
  fpfh_radius_(0.02),
  harris_thresh_(0.01),
  harris_margin_(50),
  use_3d_sift_(true),
  step_(1),
  max_z_(3)
{
  ftype_ = ORB;
  verification_type_ = MDE;
  //orb_ = boost::shared_ptr<cv::ORB>(new cv::ORB());
  view_handler_ = (GridSearchViewHandler*) this;
  prev_pcd_ = Cloud::Ptr(new Cloud);
  cur_pcd_ = Cloud::Ptr(new Cloud);
  cur_pcd_transformed_ = Cloud::Ptr(new Cloud);
  //Visualization settings set to be sensible for PS devices
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

bool LoopCloser::getLinkHypotheses(const rgbd::Frame &frame, size_t t, vector<size_t> &targets, 
    vector<Eigen::Affine3f> &transforms)
{
  //Set orb_ and sift_
  orb_ = boost::shared_ptr<cv::ORB>(new cv::ORB(keypoints_per_frame_));
  sift_ = boost::shared_ptr<cv::SIFT>(new cv::SIFT(keypoints_per_frame_));
  //Visualization
  if(visualize_)
    sseq_->model_.frameToCloud(frame, cur_pcd_.get());
  if(fine_tune_)
  {
    vector<Eigen::Affine3f> untuned;
    bool succeeded = getInitHypotheses(frame, t, targets, untuned);
    if(!succeeded) 
      return false;
    cout << "Found " << targets.size() << " possible transforms." << endl;
    transforms.resize(targets.size());
    for(size_t i = 0; i < targets.size(); i++)
    {
      Frame frame_target;
      sseq_->readFrame(targets[i], &frame_target);
      if(visualize_)
        sseq_->model_.frameToCloud(frame_target, prev_pcd_.get());
      cout << "Fine tuning" << endl;
      transforms[i] = fineTuneHypothesis(frame, frame_target, untuned[i]);
    }
  }
  else
  {
    return getInitHypotheses(frame, t, targets, transforms);
  }
  return true;

}
//! If true, returns a set of target frame indices and transforms, before fine tuning
bool LoopCloser::getInitHypotheses(const rgbd::Frame &frame, size_t t, vector<size_t> &targets, 
    vector<Eigen::Affine3f> &transforms)
{
  //Extract keypoints on current frame
  vector<cv::KeyPoint> cur_keypoints;
  FeaturesPtr cur_features = getFeatures(frame, t, cur_keypoints);
  if(cur_keypoints.size() < min_ransac_inliers_)
    return false;
  Cloud::ConstPtr cur_cloud = sseq_->getCloud(t);
  size_t latest_time = t;
  vector<bool> has_transform(cached_frames_.size(), false);
  vector<Eigen::Affine3f> all_transforms(cached_frames_.size());
#pragma omp parallel for
  for(int i = cached_frames_.size()-1; i >= 0; i-=step_)
  {
    //Lookup cached keypoints from past frames
    //ALL keypoints must exist in the cloud (z != nan) already
    size_t old_t = cached_frames_[i];
    cout << "Old_t = " << old_t << endl;
    cout << "i = " << i << endl;
    if( latest_time - old_t < min_time_offset_)
      continue;
    const vector<cv::KeyPoint> &old_keypoints = keypoint_cache_[old_t];
    FeaturesPtr old_features = feature_cache_[old_t];
    //Match them
    cv::FlannBasedMatcher matcher_;
    vector<cv::DMatch> matches_all;
    //matcher_.match(*cur_features, *old_features, matches_all);
    vector<vector<cv::DMatch> > matches_each;
    if(old_features->rows < k_ || cur_features->rows < k_) continue;
    try
    {
    matcher_.knnMatch(*cur_features, *old_features, matches_each, k_);
    }
    catch(...) {continue;}
    for(size_t j = 0; j < matches_each.size(); j++)
    {
      const vector<cv::DMatch> &matches = matches_each[j];
      for(size_t k = 0; k < matches.size(); k++)
        matches_all.push_back(matches[k]);
    }
    //Remove nonexistent ones
    vector<cv::DMatch> matches;
    for(size_t j = 0; j < matches_all.size(); j++)
    {
      if(matches_all[j].queryIdx < 0 || matches_all[j].trainIdx < 0)
        continue;
      if(matches_all[j].distance > max_feature_dist_)
        continue;
      matches.push_back(matches_all[j]);
    }
    //Check that enough are matched
    if(matches.size() < min_ransac_inliers_)
      continue;
    //Visualize
    if(visualize_)
    {
      Frame old_frame; sseq_->readFrame(old_t, &old_frame);
      cv::Mat3b img_match;
      cv::drawMatches(frame.img_, cur_keypoints, old_frame.img_, old_keypoints,
          matches, img_match);
      cv::imshow("match", img_match);
      cv::waitKey(50);
    }
    //Get cloud
    Cloud::ConstPtr old_cloud = sseq_->getCloud(old_t);
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
      pcl::PointXYZRGB r0 = cur_cloud->at(cur_keypoints[idx0].pt.x, cur_keypoints[idx0].pt.y);
      pcl::PointXYZRGB r1 = cur_cloud->at(cur_keypoints[idx1].pt.x, cur_keypoints[idx1].pt.y);
      pcl::PointXYZRGB r2 = cur_cloud->at(cur_keypoints[idx2].pt.x, cur_keypoints[idx2].pt.y);
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
      pcl::PointXYZRGB t0 = old_cloud->at(old_keypoints[oidx0].pt.x, old_keypoints[oidx0].pt.y);
      pcl::PointXYZRGB t1 = old_cloud->at(old_keypoints[oidx1].pt.x, old_keypoints[oidx1].pt.y);
      pcl::PointXYZRGB t2 = old_cloud->at(old_keypoints[oidx2].pt.x, old_keypoints[oidx2].pt.y);
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
    for(size_t j = 0; j < cur_keypoints.size(); ++j)
      keypoint_cloud0->push_back(cur_cloud->at(cur_keypoints[j].pt.x, cur_keypoints[j].pt.y));
    Cloud::Ptr keypoint_cloud1(new Cloud);
    for(size_t j = 0; j < old_keypoints.size(); ++j)
      keypoint_cloud1->push_back(old_cloud->at(old_keypoints[j].pt.x, old_keypoints[j].pt.y));

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
      //Refine
      pcl::TransformationFromCorrespondences tfc;
      for(size_t j = 0; j < best_inliers.size(); j++)
        tfc.add(keypoint_cloud1->points[best_inliers[j].first].getVector3fMap(), 
            keypoint_cloud0->points[best_inliers[j].second].getVector3fMap());
      Eigen::Affine3f trans_refined = tfc.getTransformation();
      bool invalid = false;
      switch(verification_type_)
      {
        case ICP:
          {
          //Check ICP
          float icp_score = 0;
          size_t num_total = 0;
          size_t num_inliers = 0;
          //Initialize kdtree of pointcloud
          pcl::KdTreeFLANN<Point> kdtree0_full;
          kdtree0_full.setInputCloud(cur_cloud);
          Cloud::Ptr old_cloud_transformed(new Cloud); pcl::transformPointCloud(*old_cloud, *old_cloud_transformed, trans_refined);
          int step = 5;
          for(size_t k = 0; k < old_cloud_transformed->points.size(); k+=step)
          {
            num_total++;
            if(isnan(old_cloud_transformed->points[k].z)) continue;
            indices.clear(); distances.clear();
            kdtree0_full.nearestKSearch(old_cloud_transformed->points[k], 1, indices, distances);
            if(distances.size() == 0 || distances[0] > icp_max_inlier_dist_)
              continue;
            icp_score += distances[0];
            num_inliers++;
          }
          float avg_dist = icp_score / num_inliers;
          float inlier_pct = num_inliers / (float) num_total;
          cout << "Avg icp distance = " << avg_dist << endl;
          cout << "Inlier percent = " << inlier_pct << endl;
          if(avg_dist > icp_max_avg_dist_ || inlier_pct < icp_inlier_percent_)
            invalid = true;
          }
          break;

        case MDE:
          {
          Frame frame_prev; sseq_->readFrame(old_t, &frame_prev);
	  ROS_WARN("LoopCloser::getInitHypotheses using default rather than learned model.");
          FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(sseq_->model_, frame, 
            sseq_->model_, frame_prev, max_z_, 0.25));
          double rx, ry, rz, tx, ty, tz;
          generateXYZYPR(trans_refined.inverse(), rx, ry, rz, tx, ty, tz);
          Eigen::ArrayXd trans_array(6); trans_array << rx, ry, rz, tx, ty, tz;
          double mean_dist = mde->eval(trans_array);
          cout << "Mean_dist = " << mean_dist << endl;
          if (mean_dist > max_mde_)
            invalid = true;
          }
          break;
      }
      if(invalid)
      {
        cout << "Failed verification" << endl;
        continue;
      }
      cout << "Passed Verification" << endl;
      //targets.push_back(old_t);
      //transforms.push_back(trans_refined.inverse());

      all_transforms[i] = trans_refined.inverse();      
      has_transform[i] = true;
      //latest_time = old_t; //Don't look til another N steps
    }
  }
  for(size_t i = 0; i < has_transform.size(); i++)
  {
    if(has_transform[i]){
      targets.push_back(cached_frames_[i]);
      transforms.push_back(all_transforms[i]);
    }
  }
  return targets.size() > 0;
}
//! Fine tune a hypothesis
Eigen::Affine3f LoopCloser::fineTuneHypothesis(const rgbd::Frame &frame_cur, 
    const rgbd::Frame &frame_prev, const Eigen::Affine3f &hypothesis)
{
  //FrameAligner aligner(sseq_->model_, sseq_->model_);
  //return aligner.align(frame_cur, frame_prev, hypothesis.cast<double>()).cast<float>();
  return alignFrames(frame_cur, frame_prev, hypothesis);
}
  
//! Get features from orb and cache them
LoopCloser::FeaturesPtr LoopCloser::getFeatures(const rgbd::Frame &frame, 
    size_t t, vector<cv::KeyPoint> &keypoints)
{
  cv::Mat feat;
  cv::Mat1b img;
  cv::cvtColor(frame.img_, img, CV_BGR2GRAY);
  cv::Mat1b mask = cv::Mat1b::zeros(img.rows, img.cols);
  // Only look for keypoints in regions where there is a depth reading < max_z_
  //Look up cloud
  Cloud::ConstPtr cloud = sseq_->getCloud(t);
  for(int x = 0; x < mask.cols; x++)
  {
    for(int y = 0; y < mask.rows; y++)
    {
      const pcl::PointXYZRGB &pt = cloud->at(x, y);
      if(!isnan(pt.z) && pt.z < max_z_)
        mask(y,x) = 255;
    }
  }
  if(visualize_)
    cv::imshow("Mask", mask);
  vector<cv::KeyPoint> keypoints_all;
  switch(ftype_)
  {
    case ORB:
      (*orb_)(img, mask, keypoints_all, feat);
      break;
    case SIFT:
      (*sift_)(img, mask, keypoints_all, feat);
      break;
    case FPFH:
      {
        //Get normals
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        ne.setRadiusSearch(fpfh_radius_);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*normals);
        //Get keypoints
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr keycloud(new pcl::PointCloud<pcl::PointXYZI>);
        if(use_3d_sift_)
        {
          pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI> det;
          det.setInputCloud(cloud);
          det.setScales(0.01f, 2, 2);
          det.setMinimumContrast(0.0);
          det.compute(*keycloud);
        }
        else
        {
          pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> det;
          det.setInputCloud(cloud);
          det.setMethod(det.LOWE);
          det.setSearchMethod(tree);
          det.setNonMaxSupression(true);
          det.setThreshold(harris_thresh_);
          det.compute(*keycloud);
        }
        Cloud::Ptr keycloud_rgb(new Cloud);
        for(size_t i = 0; i < keycloud->points.size(); i++)
        {
          vector<float> distances; vector<int> indices;
          pcl::PointXYZRGB pt; pt.getVector3fMap() = keycloud->points[i].getVector3fMap();
          tree->nearestKSearch(pt, 1, indices, distances);
          if(indices.size() < 1) continue;
          size_t idx = indices[0];
          size_t x = idx % cloud->width;
          size_t y = idx / cloud->width;
          if (x < harris_margin_ || x > cloud->width - harris_margin_)
            continue;
          if (y < harris_margin_ || y > cloud->height - harris_margin_)
            continue;
          assert(cloud->at(x,y).x == cloud->points[idx].x);
          cv::KeyPoint kpt;
          kpt.pt = cv::Point(x,y);
          keypoints_all.push_back(kpt);
          keycloud_rgb->push_back(cloud->points[idx]);
        }
        //Get features
        cout << "Cloud has " << keycloud_rgb->points.size() << " points" << endl;
        cout << "Surface has " << cloud->points.size() << " points" << endl;
        pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, Eigen::MatrixXf> fpfh;
        fpfh.setSearchMethod(tree);
        fpfh.setSearchSurface(cloud);
        fpfh.setRadiusSearch(fpfh_radius_);
        fpfh.setInputCloud(keycloud_rgb);
        fpfh.setInputNormals(normals);
        pcl::PointCloud<Eigen::MatrixXf> feat_cloud; fpfh.computeEigen(feat_cloud);
        //Convert to Mat
        const Eigen::MatrixXf &feat_mat = feat_cloud.points;
        feat = cv::Mat1f(feat_mat.rows(), feat_mat.cols());
        for(size_t i = 0; i < feat_mat.rows(); i++)
          for(size_t j = 0; j < feat_mat.cols(); j++)
            feat.at<float>(i,j) = feat_mat(i,j);
      }
  }
  //Eliminate all keypoints which don't have depth readings
  vector<size_t> valid;
  for(size_t i = 0; i < keypoints_all.size(); i++)
  {
    const cv::KeyPoint &kpt = keypoints_all[i];
    const pcl::PointXYZRGB &pt = cloud->at(kpt.pt.x, kpt.pt.y);
    if(!isnan(pt.z) && pt.z < max_z_)
      valid.push_back(i);
  }
  keypoints.resize(valid.size());
  FeaturesPtr feat_valid(new cv::Mat1f(keypoints.size(), feat.cols));
  for(size_t i = 0; i < valid.size(); i++)
  {
    size_t idx = valid[i];
    keypoints[i] = keypoints_all[idx];
    for(int j  = 0; j < feat.cols; j++)
    {
      if(ftype_ == ORB || ftype_ == SIFT)
        (*feat_valid)(i,j) = feat.at<uint8_t>(idx,j);
      else
        (*feat_valid)(i,j) = feat.at<float>(i,j);
    }
  }
  keypoint_cache_[t] = keypoints;
  feature_cache_[t] = feat_valid;
  cached_frames_.push_back(t);
  return feat_valid;
}

//! Align frames
Eigen::Affine3f LoopCloser::alignFrames(const rgbd::Frame &frame0, const rgbd::Frame &frame1, 
    const Eigen::Affine3f &guess)
{
  //Convert to clouds, populate trees
  //vector<Cloud::ConstPtr> clouds0;
  //vector<KdTree::Ptr> trees0;
  //Cloud::Ptr cloud0(new Cloud); sseq_->model_.frameToCloud(frame0, cloud0.get());
  //clouds0.push_back(cloud0);
  //KdTree::Ptr tree0(new KdTree);
  //tree0->setInputCloud(cloud0);
  //trees0.push_back(tree0);
  //vector<Cloud::Ptr> clouds1;
  //Cloud::Ptr cloud1(new Cloud); sseq_->model_.frameToCloud(frame1, cloud1.get());
  //clouds1.push_back(cloud1);
  //pipeline::Params params;
  //params.set("TimeCorrespondenceThreshold", numeric_limits<double>::infinity());
  //params.set("DistanceThreshold", (double) 0.03);
  //params.set("Seq0Fx", (double)0);
  //params.set("Seq0Fy", (double)0);
  //params.set("Seq0Cx", (double)0);
  //params.set("Seq0Cy", (double)0);
  //boost::shared_ptr<LossFunction> lf(new LossFunction(trees0, clouds0, clouds1, params));
  //lf->use_fsv_ = false;
  ROS_WARN_ONCE("LoopCloser does not use learned model.");
  FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(sseq_->model_, frame0, 
        sseq_->model_, frame1, max_z_, 0.25));

  GridSearch gs(6);
  gs.verbose_ = false;
  gs.view_handler_ = view_handler_;
  gs.objective_ = mde;
  gs.num_scalings_ = 5; //Was 3
  double max_res_rot = 1.5* M_PI/180;
  double max_res_trans = 0.02;
  gs.max_resolutions_ << max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 1;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  //Convert guess to XYZRPY
  double rx, ry, rz, tx, ty, tz;
  generateXYZYPR(guess, rx, ry, rz, tx, ty, tz);
  //Now search with that initialization
  Eigen::ArrayXd init(6); init << rx, ry, rz, tx, ty, tz;
  Eigen::ArrayXd x = gs.search(init);
  cout << gs.num_evals_ / gs.time_ << " evals / second." << endl;
  cout << gs.time_ / gs.num_evals_ << " seconds / eval." << endl;
  
  return generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));

}

void LoopCloser::handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective)
{
  if(!visualize_) return;
  if(!vis_.updatePointCloud(prev_pcd_, "last"))
    vis_.addPointCloud(prev_pcd_, "last");
  Eigen::Affine3f transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(4));
  cout << "Transform = " << endl << transform.matrix() << endl;
  pcl::transformPointCloud(*cur_pcd_, *cur_pcd_transformed_, transform);
  if(!vis_.updatePointCloud(cur_pcd_transformed_, "transformed"))
    vis_.addPointCloud(cur_pcd_transformed_, "transformed");
  vis_.spinOnce(3);
}
