#include <image_labeler/opencv_view.h>
#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include <agent/agent.h>
#include <timer/timer.h>
#include <boost/program_options.hpp>
#include <stream_sequence/frame_projector.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;


typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

class FeatureSet
{
public:
  Cloud::Ptr keycloud_;
  cv::Mat1b descriptors_;
  std::vector<cv::KeyPoint> keypoints_;

  void draw(cv::Mat3b img) const;
};

void FeatureSet::draw(cv::Mat3b img) const
{
  for(size_t i = 0; i < keypoints_.size(); ++i)
    cv::circle(img, keypoints_[i].pt, 2, cv::Scalar(0, 0, 255), -1);
}

// Only accepts keypoints that have 3D associated with them.
void computeFeatures(clams::Frame frame, FeatureSet* fs)
{
  // Set up projector.
  clams::FrameProjector proj;
  proj.width_ = frame.img_.cols;
  proj.height_ = frame.img_.rows;
  proj.cx_ = proj.width_ / 2;
  proj.cy_ = proj.height_ / 2;
  ROS_ASSERT(frame.img_.cols == 640);
  proj.fx_ = 525;
  proj.fy_ = 525;

  // Create mask.  Only allow points which have depth.
  cv::Mat1b mask = cv::Mat1b(frame.img_.rows, frame.img_.cols);
  mask = 255;
  for(int y = 0; y < frame.img_.rows; ++y)
    for(int x = 0; x < frame.img_.cols; ++x)
      if(frame.depth_->coeffRef(y, x) == 0)
        mask(y, x) = 0;

  cv::Mat1b gray;
  cv::cvtColor(frame.img_, gray, CV_BGR2GRAY);

  // Compute features.
  string type = "ORB";
  cv::Mat cvdesc;
  if(type == "SURF") {
    int min_hessian = 400;
    cv::SurfFeatureDetector detector(min_hessian);
    detector.detect(gray, fs->keypoints_);
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(gray, fs->keypoints_, cvdesc);
  }
  else {
    cv::ORB orb(500);  // ~500 keypoints per image
    orb(gray, mask, fs->keypoints_, cvdesc);
  }
  cout << fs->keypoints_.size() << " " << cvdesc.rows << endl;
  ROS_ASSERT(fs->keypoints_.size() == (size_t)cvdesc.rows);

  // Make keypoint cloud.
  fs->keycloud_ = Cloud::Ptr(new Cloud());
  fs->keycloud_->points.resize(fs->keypoints_.size());
  fs->keycloud_->height = 1;
  fs->keycloud_->width = fs->keypoints_.size();
  for(size_t i = 0; i < fs->keypoints_.size(); i++) {
    const cv::KeyPoint &kpt = fs->keypoints_[i];
    clams::ProjectivePoint kppt;
    kppt.u_ = kpt.pt.x;
    kppt.v_ = kpt.pt.y;
    kppt.z_ = frame.depth_->coeffRef(kppt.v_, kppt.u_);
    proj.project(kppt, &(fs->keycloud_->points[i]));
  }

  // Copy over descriptors.
  if(type == "SURF") {
    fs->descriptors_ = cv::Mat1f(fs->keypoints_.size(), cvdesc.cols);
    for(size_t i = 0; i < fs->keypoints_.size(); i++)
      for(int j  = 0; j < cvdesc.cols; j++)
        fs->descriptors_(i, j) = cvdesc.at<float>(i, j);
  }
  else {
    // ORB uses 8bit unsigned descriptors,
    // but FLANN requires floats.  Is this OK?
    // Or are the ORB descriptors only bitstrings?
    fs->descriptors_ = cv::Mat1b(fs->keypoints_.size(), cvdesc.cols);
    for(size_t i = 0; i < fs->keypoints_.size(); i++)
      for(int j  = 0; j < cvdesc.cols; j++)
        fs->descriptors_(i, j) = cvdesc.at<uint8_t>(i, j);
  }
}

//! Rigid Object Detector
class RodVisualizer : public OpenNI2Handler, public OpenCVViewDelegate, public Agent
{
public:
  RodVisualizer();
  
protected:
  OpenNI2Interface oni_;
  OpenCVView view_;
  std::vector<cv::Point2i> selected_points_;
  //! The thing we want to detect.
  FeatureSet model_;
  
  // Protected by mutex_.
  openni::VideoFrameRef current_color_;
  openni::VideoFrameRef current_depth_;
  
  // color and depth contain sensor timestamps.
  // timestamp contains the wall time, in seconds.
  void rgbdCallback(openni::VideoFrameRef color,
                    openni::VideoFrameRef depth,
                    size_t frame_id, double timestamp);
  void _run();
  void mouseEvent(int event, int x, int y, int flags, void* param);
  void keypress(char c);
  void visualizeFeatures();
  void drawSelection(cv::Mat3b vis) const;
  void detect();
  void select();
  clams::Frame getFrame();
};

RodVisualizer::RodVisualizer() :
  oni_(OpenNI2Interface::VGA, OpenNI2Interface::VGA),
  view_("Rigid Object Detector")
{
  view_.setDelegate(this);
}

void RodVisualizer::rgbdCallback(openni::VideoFrameRef color,
                                 openni::VideoFrameRef depth,
                                 size_t frame_id, double timestamp)
{
  lockWrite();
  current_color_ = color;
  current_depth_ = depth;
  unlockWrite();
  
  cv::Mat3b vis = oniToCV(color);
  drawSelection(vis);
  view_.updateImage(vis);
  char c = view_.cvWaitKey(2);
  if(c != -1)
    keypress(c);
}

void RodVisualizer::drawSelection(cv::Mat3b vis) const
{
  for(size_t i = 0; i < selected_points_.size(); ++i)
    cv::circle(vis, selected_points_[i], 4, cv::Scalar(0, 255, 0), -1);

  // Fill in box if we have the upper left and lower right corners.
  if(selected_points_.size() == 2) {
    cv::rectangle(vis, selected_points_[0], selected_points_[1], cv::Scalar(0, 255, 0), 1);
  }
}

void RodVisualizer::mouseEvent(int event, int x, int y, int flags, void* param)
{
  if(event == CV_EVENT_LBUTTONUP) {
    if(selected_points_.size() == 2)
      selected_points_.clear();
    
    selected_points_.push_back(cv::Point2i(x, y));
  }
}

void RodVisualizer::_run()
{
  oni_.setHandler(this);
  oni_.run();
}

void RodVisualizer::keypress(char c)
{
  switch(c) {
  case 'q':
    oni_.stop();
    break;
  case 'f':
    visualizeFeatures();
    break;
  case 's':
    select();
    break;
  case 'd':
    cout << "============================================================" << endl;
    detect();
    break;
  default:
    break;
  }
}

void RodVisualizer::select()
{
  if(selected_points_.size() != 2) {
    cout << "You must select the template first." << endl;
    return;
  }

  // Compute features on the entire image.
  clams::Frame frame = getFrame();
  FeatureSet fs;
  computeFeatures(frame, &fs);
  cv::Point2i ul = selected_points_[0];
  cv::Point2i lr = selected_points_[1];  

  // Get the FeatureSet for the object model.
  model_.keypoints_.clear();
  model_.keypoints_.reserve(fs.keypoints_.size());
  model_.keycloud_ = Cloud::Ptr(new Cloud);
  model_.keycloud_->clear();
  model_.keycloud_->reserve(fs.keypoints_.size());
  vector<int> indices;
  indices.reserve(fs.keypoints_.size());
  for(size_t i = 0; i < fs.keypoints_.size(); ++i) {
    cv::Point2f kpt = fs.keypoints_[i].pt;
    if(kpt.x > ul.x && kpt.x < lr.x &&
       kpt.y > ul.y && kpt.y < lr.y)
    {
      model_.keypoints_.push_back(fs.keypoints_[i]);
      model_.keycloud_->push_back(fs.keycloud_->at(i));
      indices.push_back(i);
    }
  }
  model_.descriptors_ = cv::Mat1b(model_.keypoints_.size(), fs.descriptors_.cols);
  for(int i = 0; i < model_.descriptors_.rows; ++i)
    for(int j  = 0; j < model_.descriptors_.cols; j++)
      model_.descriptors_(i, j) = fs.descriptors_(indices[i], j);

  // Display.
  cv::Mat3b vis = frame.img_.clone();
  model_.draw(vis);
  cv::imshow("Model", vis);
}

void sampleCorrespondence(const std::vector<cv::DMatch>& matches,
                          const FeatureSet& model, const FeatureSet& image,
                          size_t* model_idx, size_t* image_idx)
{
  while(true) {
    const cv::DMatch& m = matches[rand() % matches.size()];
    *model_idx = m.trainIdx;
    *image_idx = m.queryIdx;

    // Only accept if both points have depth.
    if(isFinite(model.keycloud_->at(*model_idx)) &&
       isFinite(image.keycloud_->at(*image_idx)))
    {
      break;
    }
  }

}

void search(const FeatureSet& model, const FeatureSet& image, vector<FeatureSet>* detections)
{
  detections->clear();
  
}

FeatureSet search(const FeatureSet& model, const FeatureSet& image)
{
  ROS_ASSERT(!model.keypoints_.empty());
  ROS_ASSERT(!image.keypoints_.empty());

  // Params.
  // TODO: These should elsewhere.  This function should probably be in its own object
  // and have a YAML for configuration.
  int k = 1;
  int max_consecutive_nondetections = 1e5;
  float inlier_distance_thresh = 0.01;  // cm
  int descriptor_distance_thresh = 50;
 
  // Compute matches.
  //cv::FlannBasedMatcher matcher;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  vector< vector<cv::DMatch> > matches_raw;  // matches_raw[i][j] is the jth model feature match for the ith image feature.
  matcher.knnMatch(image.descriptors_, model.descriptors_, matches_raw, k);

  // Print out stats on the matches.
  double max_descriptor_distance = -std::numeric_limits<double>::max();
  double min_descriptor_distance = +std::numeric_limits<double>::max();
  double total_descriptor_distance = 0;
  int num_matches = 0;
  for(size_t i = 0; i < matches_raw.size(); ++i) {
    if(matches_raw[i].empty())
      continue;

    // Check that matches go from best to worst.
    for(size_t j = 1; j < matches_raw[i].size(); ++j) {
      ROS_ASSERT(matches_raw[i][j].distance >= matches_raw[i][j-1].distance);
    }
    
    const cv::DMatch& match = matches_raw[i][0];
    max_descriptor_distance = max<double>(max_descriptor_distance, match.distance);
    min_descriptor_distance = min<double>(min_descriptor_distance, match.distance);

    total_descriptor_distance += match.distance;
    ++num_matches;

    cout << "descriptor distance: " << match.distance << endl;
  }
  cout << "Mean descriptor distance to match: " << total_descriptor_distance / num_matches << endl;
  cout << "Min descriptor distance: " << min_descriptor_distance << endl;
  cout << "Max descriptor distance: " << max_descriptor_distance << endl;
  
  // Throw out crappy matches.
  size_t num_good = 0;
  vector<cv::DMatch> matches;
  matches.reserve(num_matches);
  for(size_t i = 0; i < matches_raw.size(); ++i) {
    for(size_t j = 0; j < matches_raw[i].size(); ++j) {
      if(matches_raw[i][j].distance < descriptor_distance_thresh) {
        matches.push_back(matches_raw[i][j]);
      }
    }
  }
  cout << "Good matches: " << matches.size() << endl;
  if(matches.size() < model.keycloud_->size() / 5)
    return FeatureSet();
  
  int best_num_inliers = 0;
  int num_samples_without_detection = 0;
  while(true) {    
    // If we haven't found anything in a while, stop.
    //cout << "num_samples_without_detection: " << num_samples_without_detection << endl;
    ++num_samples_without_detection;
    if(num_samples_without_detection > max_consecutive_nondetections)
      break;

    // Sample a possible set of three matches.
    // They should all have depth because of the mask applied in computeFeatures.
    // cout << model.keypoints_.size() << " " << model.keycloud_->size() << endl;
    // cout << image.keypoints_.size() << " " << image.keycloud_->size() << endl;
    size_t model_idx0, image_idx0;
    size_t model_idx1, image_idx1;
    size_t model_idx2, image_idx2;
    sampleCorrespondence(matches, model, image, &model_idx0, &image_idx0);
    sampleCorrespondence(matches, model, image, &model_idx1, &image_idx1);
    sampleCorrespondence(matches, model, image, &model_idx2, &image_idx2);
    ROS_ASSERT(model_idx0 < model.keycloud_->size() && image_idx0 < image.keycloud_->size());
    ROS_ASSERT(model_idx1 < model.keycloud_->size() && image_idx1 < image.keycloud_->size());
    ROS_ASSERT(model_idx2 < model.keycloud_->size() && image_idx2 < image.keycloud_->size());
    Point m0 = model.keycloud_->at(model_idx0);
    Point m1 = model.keycloud_->at(model_idx1);
    Point m2 = model.keycloud_->at(model_idx2);
    Point i0 = image.keycloud_->at(image_idx0);
    Point i1 = image.keycloud_->at(image_idx1);
    Point i2 = image.keycloud_->at(image_idx2);
    // cout << m0 << " " << i0 << endl;
    // cout << m1 << " " << i1 << endl;
    // cout << m2 << " " << i2 << endl;
    ROS_ASSERT(isFinite(m0) && isFinite(i0));
    ROS_ASSERT(isFinite(m1) && isFinite(i1));
    ROS_ASSERT(isFinite(m2) && isFinite(i2));

    // If the sampled points are too close together, don't bother.
    // Re-using the inlier_distance_thresh.
    double d01 = pcl::euclideanDistance(i0, i1);
    double d02 = pcl::euclideanDistance(i0, i2);
    double d12 = pcl::euclideanDistance(i1, i2);
    if(d01 < inlier_distance_thresh ||
       d02 < inlier_distance_thresh ||
       d12 < inlier_distance_thresh)
    {
      //cout << "Sampled points are too close." << endl;
      continue;
    }
    
    // Compute the transform from model to image for this match.
    // Model to image.
    pcl::TransformationFromCorrespondences tfc;
    tfc.add(m0.getVector3fMap(), i0.getVector3fMap());
    tfc.add(m1.getVector3fMap(), i1.getVector3fMap());
    tfc.add(m2.getVector3fMap(), i2.getVector3fMap());
    Eigen::Affine3f trans = tfc.getTransformation();
    
    if((trans * m0.getVector3fMap() - i0.getVector3fMap()).norm() > inlier_distance_thresh ||
       (trans * m1.getVector3fMap() - i1.getVector3fMap()).norm() > inlier_distance_thresh ||
       (trans * m2.getVector3fMap() - i2.getVector3fMap()).norm() > inlier_distance_thresh)
    {
      //cout << "Inconsistent correspondences, skipping." << endl;
      continue;
    }

    // Transform the model into the image.
    Cloud transformed_model_keycloud;
    pcl::transformPointCloud(*model.keycloud_, transformed_model_keycloud, trans);

    // Find number of inliers.
    // For every image keypoint, find the distance to its matches in the
    // transformed model.
    size_t num_rough_inliers = 0;
    pcl::TransformationFromCorrespondences tfc2;
    for(size_t i = 0; i < matches.size(); ++i) {
      const cv::DMatch& match = matches[i];
      Point mpt = transformed_model_keycloud[match.trainIdx];
      Point ipt = image.keycloud_->at(match.queryIdx);
      float dist = pcl::euclideanDistance(mpt, ipt);
      if(dist < inlier_distance_thresh) {
        tfc2.add(model.keycloud_->at(match.trainIdx).getVector3fMap(), ipt.getVector3fMap());
        ++num_rough_inliers;
      }
    }
    double rough_inlier_pct = (double)num_rough_inliers / model.keycloud_->size();
    cout << "rough_inlier_pct: " << rough_inlier_pct << " --- total " << model.keycloud_->size() << endl;
    if(rough_inlier_pct < 0.10)
      continue;
    
    // Transform the model into the image using the refined cloud.
    Eigen::Affine3f trans_refined = tfc2.getTransformation();
    pcl::transformPointCloud(*model.keycloud_, transformed_model_keycloud, trans_refined);

    // Get inliers after refinement.
    vector<size_t> inlier_indices;
    inlier_indices.reserve(model.keycloud_->size());
    FeatureSet detection;
    for(size_t i = 0; i < matches.size(); ++i) {
      const cv::DMatch& match = matches[i];
      Point mpt = transformed_model_keycloud[match.trainIdx];
      Point ipt = image.keycloud_->at(match.queryIdx);
      float dist = pcl::euclideanDistance(mpt, ipt);
      if(dist < inlier_distance_thresh) {
        inlier_indices.push_back(match.queryIdx);
        detection.keypoints_.push_back(image.keypoints_[match.queryIdx]);
      }
    }
    
    double inlier_pct = (double)inlier_indices.size() / model.keycloud_->size();
    cout << "inlier_pct: " << inlier_pct << " --- total " << model.keycloud_->size() << endl;
    if(inlier_pct > 0.20) {
      return detection;
    }
  }

  return FeatureSet();  // No detection.
}

//   // -- Get best correspondences for use in grid search and build up flat vector of all matches
//   //    that meet our criteria.
//   vector<cv::DMatch> matches;
//   correspondences0->reserve(matches_each.size());
//   correspondences1->reserve(matches_each.size());
//   double max_feature_dist = params_.get<double>("max_feature_dist");
//   for(size_t j = 0; j < matches_each.size(); j++) {
//     const vector<cv::DMatch>& mat = matches_each[j];
//     if(mat.empty())
//       continue;

//     bool added_corr = false;
//     for(size_t k = 0; k < mat.size(); k++) {
//       //cout << mat[k].distance << " ";
//       if(mat[k].queryIdx < 0 || mat[k].trainIdx < 0)
//         continue;
//       if(mat[k].distance > max_feature_dist)
//         continue;

//       // Make sure opencv is doing what we think it is.
//       if(k > 0)
//         ROS_ASSERT(mat[k].distance >= mat[k-1].distance);
      
//       // Correspondences get filled with the best matches that are at least somewhat good.
//       // They don't need depth.
//       if(!added_corr) {
//         correspondences0->push_back(keypoints0[mat[k].queryIdx].pt);
//         correspondences1->push_back(keypoints1[mat[k].trainIdx].pt);
//         added_corr = true;
//       }

//       // Saved matches must have depth for 3d ransac.
//       /*  
//       if(frame0.depth_->coeffRef(keypoints0[mat[k].queryIdx].pt.y, keypoints0[mat[k].queryIdx].pt.x) > 0 &&
//          frame1.depth_->coeffRef(keypoints1[mat[k].trainIdx].pt.y, keypoints1[mat[k].trainIdx].pt.x) > 0)
//       {
//         matches.push_back(mat[k]);
//       }
//       */
//       if( isFinite(keycloud0->at(mat[k].queryIdx)) && 
//           isFinite(keycloud1->at(mat[k].trainIdx)) )
//       {
//         matches.push_back(mat[k]);
//       }
//     }
//   }

//   //Check that enough are matched
//   if(matches.size() < (size_t)params_.get<int>("min_ransac_inliers")) {
//     ROS_DEBUG_STREAM(matches.size() << " is not enough matches, skipping.");
//     return false;
//   }
// // SDM Removed for now, trying to avoid loading frames
// //#ifdef VISUALIZE
// //  cv::Mat3b img_match;
// //  cv::drawMatches(frame0.img_, keypoints0, frame1.img_, keypoints1,
// //                  matches, img_match);
// //  cv::imshow("match", img_match);
// //  // cv::imshow("frame0", frame0.img_);
// //  // cv::imshow("frame1", frame1.img_);
// //  cv::waitKey(500);
// //#endif

//   // TODO: Don't need to project the whole frame here, just keypoints.
//   //Cloud::Ptr cloud0(new Cloud);
//   //Cloud::Ptr cloud1(new Cloud);
//   //model0_.frameToCloud(frame0, cloud0.get());
//   //model1_.frameToCloud(frame1, cloud1.get());

//   //From here the logic is more or less copied from orb_matcher
//   vector<Eigen::Affine3f> candidates;
//   int num_ransac_samples = params_.get<int>("num_ransac_samples");
//   double min_pairwise_keypoint_dist = params_.get<double>("min_pairwise_keypoint_dist");
//   double ransac_max_inlier_dist = params_.get<double>("ransac_max_inlier_dist");
//   for(int j = 0; j < num_ransac_samples; ++j)
//   {
//     //Sample 3 points in ref img and precompute their distances to each other
//     const cv::DMatch &match0 = matches[rand() % matches.size()];
//     const cv::DMatch &match1 = matches[rand() % matches.size()];
//     const cv::DMatch &match2 = matches[rand() % matches.size()];
//     int idx0 = match0.queryIdx;
//     int idx1 = match1.queryIdx;
//     int idx2 = match2.queryIdx;
//     //pcl::PointXYZRGB r0 = cloud0->at(keypoints0[idx0].pt.x, keypoints0[idx0].pt.y);
//     //pcl::PointXYZRGB r1 = cloud0->at(keypoints0[idx1].pt.x, keypoints0[idx1].pt.y);
//     //pcl::PointXYZRGB r2 = cloud0->at(keypoints0[idx2].pt.x, keypoints0[idx2].pt.y);
//     pcl::PointXYZRGB r0 = keycloud0->at(idx0);
//     pcl::PointXYZRGB r1 = keycloud0->at(idx1);
//     pcl::PointXYZRGB r2 = keycloud0->at(idx2);
//     ROS_ASSERT(isFinite(r0) && isFinite(r1) && isFinite(r2));
//     double d01 = pcl::euclideanDistance(r0, r1);
//     double d02 = pcl::euclideanDistance(r0, r2);
//     double d12 = pcl::euclideanDistance(r1, r2);
//     if(d01 < min_pairwise_keypoint_dist ||
//        d02 < min_pairwise_keypoint_dist ||
//        d12 < min_pairwise_keypoint_dist)
//     {
//       //ROS_DEBUG_STREAM("Skipping sampled correspondences because inter-point distance is too small.");
//       continue;
//     }
//     //Get their corresponding matches
//     int oidx0 = match0.trainIdx;
//     int oidx1 = match1.trainIdx;
//     int oidx2 = match2.trainIdx;
//     //pcl::PointXYZRGB t0 = cloud1->at(keypoints1[oidx0].pt.x, keypoints1[oidx0].pt.y);
//     //pcl::PointXYZRGB t1 = cloud1->at(keypoints1[oidx1].pt.x, keypoints1[oidx1].pt.y);
//     //pcl::PointXYZRGB t2 = cloud1->at(keypoints1[oidx2].pt.x, keypoints1[oidx2].pt.y);
//     pcl::PointXYZRGB t0 = keycloud1->at(oidx0);
//     pcl::PointXYZRGB t1 = keycloud1->at(oidx1);
//     pcl::PointXYZRGB t2 = keycloud1->at(oidx2);
//     ROS_ASSERT(isFinite(t0) && isFinite(t1) && isFinite(t2));
//     //If the matches are too distant from themselves w.r.t the distance in the current frame, continue
//     if(fabs(pcl::euclideanDistance(t0, t1) - d01) > ransac_max_inlier_dist ||
//        fabs(pcl::euclideanDistance(t0, t2) - d02) > ransac_max_inlier_dist ||
//        fabs(pcl::euclideanDistance(t1, t2) - d12) > ransac_max_inlier_dist)
//     {
//       //ROS_DEBUG_STREAM("Skipping sampled correspondences because inter-point distances are too different between the two frames.");
//       continue;
//     }
//     //Otherwise generate transformation from correspondence
//     pcl::TransformationFromCorrespondences tfc;
//     tfc.add(t0.getVector3fMap(), r0.getVector3fMap());
//     tfc.add(t1.getVector3fMap(), r1.getVector3fMap());
//     tfc.add(t2.getVector3fMap(), r2.getVector3fMap());
    
//     Eigen::Affine3f trans = tfc.getTransformation();
//     if((trans * t0.getVector3fMap() - r0.getVector3fMap()).norm() > 0.01 ||
//        (trans * t1.getVector3fMap() - r1.getVector3fMap()).norm() > 0.01 ||
//        (trans * t2.getVector3fMap() - r2.getVector3fMap()).norm() > 0.01)
//       continue;
//     candidates.push_back(trans);
//   }
//   cout << "Have " << candidates.size() << " candidates after RANSAC, before inlier check" << endl;
//   //Do inlier check
//   // SDM for now leaving this in to filter out infinite points
//   Cloud::Ptr keypoint_cloud0(new Cloud);
//   for(size_t j = 0; j < keypoints0.size(); ++j) {
//     Point pt = keycloud0->at(j);
//     if(isFinite(pt))
//       keypoint_cloud0->push_back(pt);
//   }
//   Cloud::Ptr keypoint_cloud1(new Cloud);
//   for(size_t j = 0; j < keypoints1.size(); ++j) {
//     Point pt = keycloud1->at(j);
//     if(isFinite(pt))
//       keypoint_cloud1->push_back(pt);
//   }
  
//   pcl::KdTreeFLANN<Point> kdtree0;
//   kdtree0.setInputCloud(keypoint_cloud0);
  
//   Cloud transformed_keypoint_cloud1;
//   vector<int> indices;
//   vector<float> distances;
//   Eigen::Affine3f best_transform;
//   float best_distance = std::numeric_limits<float>::infinity();
//   int best_num_inliers = 0;
//   bool has_best = false;
//   vector< std::pair<size_t, size_t> > best_inliers;
//   for(size_t j = 0; j < candidates.size(); ++j)
//   {
//     float inlier_distance = 0;
//     //Ensure that, in at least 2 directions, there's variance here
//     float minx=std::numeric_limits<float>::infinity();
//     float maxx=-std::numeric_limits<float>::infinity();
//     float miny=std::numeric_limits<float>::infinity();
//     float maxy=-std::numeric_limits<float>::infinity();
//     float minz=std::numeric_limits<float>::infinity();
//     float maxz=-std::numeric_limits<float>::infinity();
//     pcl::transformPointCloud(*keypoint_cloud1, transformed_keypoint_cloud1, candidates[j]);
//     int num_inliers = 0;
//     vector< std::pair<size_t, size_t> > inliers;
//     for(size_t k = 0; k < transformed_keypoint_cloud1.size(); ++k)
//     {
//       indices.clear();
//       distances.clear();
//       kdtree0.nearestKSearch(transformed_keypoint_cloud1.points[k], 1, indices, distances);
//       if(distances.size() != 0 && distances[0] < ransac_max_inlier_dist)
//       {
//         ++num_inliers;
//         inlier_distance += distances[0];
//         inliers.push_back(std::pair<size_t, size_t>(k, indices[0]));
//         const pcl::PointXYZRGB& kpt = transformed_keypoint_cloud1.points[k];
//         //Update bounding volume
//         if(kpt.x < minx) minx = kpt.x;
//         if(kpt.x > maxx) maxx = kpt.x;
//         if(kpt.y < miny) miny = kpt.y;
//         if(kpt.y > maxy) maxy = kpt.y;
//         if(kpt.z < minz) minz = kpt.z;
//         if(kpt.z > maxz) maxz = kpt.z;
//       }
//     }
//     //Check number of inliers
//     if(num_inliers < params_.get<double>("min_ransac_inlier_percent") * keypoint_cloud0->size()) {
//       //ROS_DEBUG_STREAM("Inlier percent " << (double)num_inliers / keypoint_cloud0->size() << " is too low.");
//       continue;
//     }
//     if(num_inliers < params_.get<int>("min_ransac_inliers")) {
//       //ROS_DEBUG_STREAM(num_inliers << " is too few inliers.");
//       continue;
//     }

//     // -- Do PCA check.
//     Vector3d mean = Vector3d::Zero();
//     for(size_t i = 0; i < transformed_keypoint_cloud1.size(); ++i)
//       mean += transformed_keypoint_cloud1[i].getVector3fMap().cast<double>();
//     mean /= (double)transformed_keypoint_cloud1.size();
//     Matrix3d xxt = Matrix3d::Zero();
//     for(size_t i = 0; i < transformed_keypoint_cloud1.size(); ++i) {
//       Vector3d pt = transformed_keypoint_cloud1[i].getVector3fMap().cast<double>() - mean;
//       xxt += pt * pt.transpose();
//     }
//     xxt /= (double)transformed_keypoint_cloud1.size();
//     Eigen::JacobiSVD<Matrix3d> svd(xxt, Eigen::ComputeFullU | Eigen::ComputeFullV);
//     Matrix3d U = svd.matrixU();
//     bool passed_pca = true;
//     // Only check for variation in the first two principal components.
//     // It's ok to have lots of points defining a plane; the problem is when they're all on
//     // a line.
//     for(int i = 0; i < U.cols() - 1; ++i) {
//       Vector3d vec = U.col(i);
//       double maxval = -numeric_limits<double>::max();
//       double minval = numeric_limits<double>::max();
//       for(size_t k = 0; k < transformed_keypoint_cloud1.size(); ++k) {
//         Vector3d pt = transformed_keypoint_cloud1[k].getVector3fMap().cast<double>() - mean;
//         double val = vec.dot(pt);
//         maxval = max(val, maxval);
//         minval = min(val, minval);
//       }
//       if(maxval - minval < params_.get<double>("min_bounding_length")) {
//         // ROS_WARN_STREAM("Rejecting this hypothesized transform due to PCA check.");
//         // ROS_WARN_STREAM("Extremal values of " << minval << " to " << maxval);
//         // ROS_WARN_STREAM("Distance of " << maxval - minval << " along principal component " << i << ", " << vec.transpose());
//         // ROS_WARN_STREAM("Num pts: " << transformed_keypoint_cloud1.size());
//         passed_pca = false;
//       }
//       //ROS_DEBUG_STREAM("Distance of " << maxval - minval << " along principal component " << i << ", " << vec.transpose());
//     }
//     if(!passed_pca) {
//       //ROS_DEBUG_STREAM("PCA test failed.");
//       continue;
//     }
         
//     //Check bounding volume
//     // int has_x = (maxx - minx) > min_bounding_length_;
//     // int has_y = (maxy - miny) > min_bounding_length_;
//     // int has_z = (maxz - minz) > min_bounding_length_;
//     // if(has_x + has_y + has_z < 2)
//     // {
//     //   continue;
//     // }
    
//     // Consider this transform
//     // inlier_distance /= num_inliers; //Average out
//     // if(inlier_distance < best_distance)
//     // {
//     if(num_inliers > best_num_inliers) {
//       best_transform = candidates[j];
//       best_distance = inlier_distance;
//       best_num_inliers = num_inliers;
//       best_inliers = inliers;
//       has_best = true;

//       cout << "Best so far: " << endl;
//       cout << "  Num inliers: " << num_inliers << endl;
//       cout << "  Has bounding size of x: " << maxx - minx << ", y: " << maxy - miny << ", z: " << maxz - minz << endl;
//       cout << "  f1_to_f0: " << endl << best_transform.matrix() << endl;
//     }
//   }
//   if(has_best)
//   {
//     // Re-compute the transform using only the inliers.
//     pcl::TransformationFromCorrespondences tfc;
//     for(size_t j = 0; j < best_inliers.size(); j++)
//       tfc.add(keypoint_cloud1->points[best_inliers[j].first].getVector3fMap(), 
//               keypoint_cloud0->points[best_inliers[j].second].getVector3fMap());
//     Affine3d f1_to_f0 = tfc.getTransformation().cast<double>();
//     *f0_to_f1 = f1_to_f0.inverse();
//     cout << "Final f1_to_f0: " << endl;
//     cout << f1_to_f0.matrix() << endl;
//     return true;
//   }
//   else
//     return false;

// }

void RodVisualizer::detect()
{
  if(model_.keypoints_.empty()) {
    cout << "You must select a model using 's' first." << endl;
    return;
  }

  // Compute features on the current frame.
  clams::Frame frame = getFrame();
  FeatureSet fs_image;
  computeFeatures(frame, &fs_image);

  // Search for the model.
  //vector<FeatureSet> detections;
  //search(model_, fs_image, &detections);
  FeatureSet detection = search(model_, fs_image);

  // Debugging: Compute features again and try to match them.
  // usleep(1e6);
  // frame = getFrame();
  // FeatureSet fs_image2;
  // computeFeatures(frame, &fs_image2);
  // FeatureSet detection = search(fs_image, fs_image2);

  
  // Visualize detections.
  cv::Mat3b vis = frame.img_.clone();
  detection.draw(vis);
  cv::imshow("Detection", vis);
}

clams::Frame RodVisualizer::getFrame()
{
  clams::Frame frame;
  lockRead();
  frame.img_ = oniToCV(current_color_);
  frame.depth_ = oniDepthToEigenPtr(current_depth_);
  unlockRead();
  return frame;
}

void RodVisualizer::visualizeFeatures()
{
  clams::Frame frame = getFrame();
  FeatureSet fs;
  HighResTimer hrt("Feature computation"); hrt.start();
  computeFeatures(frame, &fs);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  cv::Mat3b vis = frame.img_.clone();
  fs.draw(vis);
  cv::imshow("Features", vis);
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  RodVisualizer rodvis;
  rodvis.run();
  
  return 0;
}
