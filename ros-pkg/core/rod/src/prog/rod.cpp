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
#include <bag_of_tricks/next_path.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

#define SAVE_IMGS (getenv("SAVE_IMGS") ? atoi(getenv("SAVE_IMGS")) : 0)

clams::FrameProjector defaultProjector(clams::Frame frame)
{
  clams::FrameProjector proj;
  proj.width_ = frame.img_.cols;
  proj.height_ = frame.img_.rows;
  proj.cx_ = proj.width_ / 2;
  proj.cy_ = proj.height_ / 2;
  ROS_ASSERT(frame.img_.cols == 640);
  proj.fx_ = 525;
  proj.fy_ = 525;
  return proj;
}

cv::Mat1b defaultCanny(cv::Mat3b img)
{
  cv::Mat1b gray;
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  return defaultCanny(gray);
}

cv::Mat1b defaultCanny(cv::Mat1b gray)
{
  cv::Mat1b canny;
  cv::Canny(gray, canny, 75, 100);
  return canny;
}

class FeatureSet
{
public:
  Cloud::Ptr keycloud_;
  cv::Mat1b descriptors_;
  std::vector<cv::KeyPoint> keypoints_;
  Cloud::Ptr pcd_;

  void drawKeypoints(cv::Mat3b img) const;
  void drawCloud(cv::Mat3b img) const;
};

void FeatureSet::drawKeypoints(cv::Mat3b img) const
{
  for(size_t i = 0; i < keypoints_.size(); ++i)
    cv::circle(img, keypoints_[i].pt, 2, cv::Scalar(0, 0, 255), -1);
}

void FeatureSet::drawCloud(cv::Mat3b img) const
{
  ROS_ASSERT(pcd_);

  clams::Frame frame; frame.img_ = img;
  clams::FrameProjector proj = defaultProjector(frame);

  clams::ProjectivePoint ppt;
  for(size_t i = 0; i < pcd_->size(); ++i) {
    proj.project(pcd_->at(i), &ppt);
    if(ppt.u_ <  0 || ppt.u_ >= img.cols)
      continue;
    if(ppt.v_ <  0 || ppt.v_ >= img.rows)
      continue;
    img(ppt.v_, ppt.u_)[0] = ppt.b_;
    img(ppt.v_, ppt.u_)[1] = ppt.g_;
    img(ppt.v_, ppt.u_)[2] = ppt.r_;
  }
}

// Only accepts keypoints that have 3D associated with them.
void computeFeatures(clams::Frame frame, FeatureSet* fs, int num_desired_keypoints = 500, cv::Mat1b mask = cv::Mat1b())
{
  clams::FrameProjector proj = defaultProjector(frame);

  // If no mask has been provided, then use the whole frame.
  if(mask.rows == 0) {
    mask = cv::Mat1b(frame.img_.rows, frame.img_.cols);
    mask = 255;
  }
  
  // Only allow points with depth.
  for(int y = 0; y < frame.img_.rows; ++y)
    for(int x = 0; x < frame.img_.cols; ++x)
      if(frame.depth_->coeffRef(y, x) == 0)
        mask(y, x) = 0;

  // cv::imshow("mask", mask);
  // cv::imshow("depth image", frame.depthImage());
  // cv::waitKey();

  cv::Mat1b gray;
  cv::cvtColor(frame.img_, gray, CV_BGR2GRAY);
  cv::Mat1b canny = defaultCanny(gray);
  cv::imshow("Canny", canny);

  // Compute features.
  string type = "ORB";
  cv::Mat cvdesc;
  if(type == "SURF") {
    int min_hessian = 400;
    cv::SurfFeatureDetector detector(min_hessian);
    detector.detect(canny, fs->keypoints_);
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(canny, fs->keypoints_, cvdesc);
  }
  else {
    cv::ORB orb(num_desired_keypoints);
    orb(canny, mask, fs->keypoints_, cvdesc);
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
    // if(kppt.z_ == 0)
    //   ROS_WARN_STREAM("Expected all keypoints to have depth, but this is apparently not true.  Keypoint: " << kpt.pt << " , rounded: " << kppt);
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

  // Debugging.
  cv::Mat3b vis = frame.img_.clone();
  fs->drawKeypoints(vis);
  cv::imshow("computeFeatures", vis);
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

  cv::imshow("Depth", colorize(oniDepthToEigen(depth), 0.5, 5));
  
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
  if(SAVE_IMGS) {
    cv::Mat3b vis = getFrame().img_.clone();
    drawSelection(vis);
    cv::imwrite(nextPath(".", "", "-initial_segmentation.png", 4), vis);
  }
  
  if(selected_points_.size() != 2) {
    cout << "You must select the template first." << endl;
    return;
  }

  // Create the mask.
  cv::Point2i ul = selected_points_[0];
  cv::Point2i lr = selected_points_[1];  
  clams::Frame frame = getFrame();
  cv::Mat1b mask(frame.img_.size());
  mask = 0;
  for(int y = 0; y < mask.rows; ++y)
    for(int x = 0; x < mask.cols; ++x)
      if(x > ul.x && x < lr.x && y > ul.y && y < lr.y)
        mask(y, x) = 255;
  
  // Compute features on just the selected region.
  FeatureSet fs;
  computeFeatures(frame, &fs, 500, mask);

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
    // ORB returns keypoints that are outside the mask?
    float slop = 10;
    if(kpt.x > ul.x - slop && kpt.x < lr.x + slop &&
       kpt.y > ul.y - slop && kpt.y < lr.y + slop)
    {
      model_.keypoints_.push_back(fs.keypoints_[i]);
      model_.keycloud_->push_back(fs.keycloud_->at(i));
      indices.push_back(i);
    }
    else {
      cout << "Invalid keypoint: " << kpt << endl;
      cout << "Upper left: " << ul << endl;
      cout << "Lower right: " << lr << endl;
      ROS_ASSERT(0);
    }
  }
  model_.descriptors_ = cv::Mat1b(model_.keypoints_.size(), fs.descriptors_.cols);
  for(int i = 0; i < model_.descriptors_.rows; ++i)
    for(int j  = 0; j < model_.descriptors_.cols; j++)
      model_.descriptors_(i, j) = fs.descriptors_(indices[i], j);

  // Also get the raw pointcloud of the object.
  // Color with a blurred edge image which we can match to later.
  cv::Mat1b gray;
  cv::cvtColor(frame.img_, gray, CV_BGR2GRAY);
  cv::Mat1b canny = defaultCanny(gray);
  clams::FrameProjector proj = defaultProjector(frame);
  model_.pcd_ = Cloud::Ptr(new Cloud);
  model_.pcd_->reserve(640*480 / 10);
  for(int y = ul.y; y < lr.y; ++y) {
    for(int x = ul.x; x < lr.x; ++x) { 
      clams::ProjectivePoint ppt;
      ppt.u_ = x;
      ppt.v_ = y;
      ppt.z_ = frame.depth_->coeffRef(ppt.v_, ppt.u_);
      if(ppt.z_ == 0)
        continue;
      Point pt;
      proj.project(ppt, &pt);
      // pt.b = frame.img_(y, x)[0];
      // pt.g = frame.img_(y, x)[1];
      // pt.r = frame.img_(y, x)[2];
      pt.b = canny(y, x);
      pt.g = canny(y, x);
      pt.r = canny(y, x);
      model_.pcd_->push_back(pt);
    }
  }

  // Display.
  cv::Mat3b vis = frame.img_.clone();
  model_.drawKeypoints(vis);
  cv::imshow("Model Keypoints", vis);
  if(SAVE_IMGS)
    cv::imwrite(nextPath(".", "", "-model_keypoints.png", 4), vis);

  selected_points_.clear();
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

struct SearchStats
{
  int num_samples_;
  int num_not_too_close_;
  vector<double> num_rough_inliers_;
  vector<double> num_refined_inliers_;
  vector<double> edge_match_pcts_;
  
  SearchStats() :
    num_samples_(0),
    num_not_too_close_(0)
  {
  }

  std::string status(const std::string& prefix = "") const;

protected:
  Eigen::VectorXd computeHistogram(const std::vector<double>& vals, double* maxval) const;
  std::string printHistogram(const std::string& prefix,
                             const std::vector<double>& vals,
                             int precision) const;
};

VectorXd SearchStats::computeHistogram(const std::vector<double>& vals, double* maxval) const
{
  *maxval = -numeric_limits<double>::max();
  for(size_t i = 0; i < vals.size(); ++i)
    *maxval = max(*maxval, vals[i]);
  int num_bins = 10;
  float bin_width = (float)(*maxval) / num_bins;
  VectorXd hist = VectorXd::Zero(num_bins);
  for(size_t i = 0; i < vals.size(); ++i) {
    int idx = min<int>(hist.rows() - 1, max<int>(0, vals[i] / bin_width));
    ++hist(idx);
  }
  return hist;
}

std::string SearchStats::printHistogram(const std::string& prefix,
                                        const std::vector<double>& vals,
                                        int precision = 0) const
{
  double maxval;
  VectorXd hist = computeHistogram(vals, &maxval);
  float bin_width = (float)maxval / hist.rows();

  ostringstream oss;
  for(int i = 0; i < hist.rows(); ++i)
    oss << prefix << setiosflags(ios::fixed) << setprecision(precision) << setw(5) << setfill('0') << i * bin_width << "\t|  " << hist[i] << endl;

  return oss.str();
}

std::string SearchStats::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "Total samples: " << num_samples_ << endl;
  oss << prefix << "Num not too close: " << num_not_too_close_ << endl;
  oss << prefix << "Num consistent: " << num_rough_inliers_.size() << endl;
  oss << prefix << "Num that passed rough inlier check: " << num_refined_inliers_.size() << endl;

  if(!num_rough_inliers_.empty()) {
    oss << prefix << "Histogram of num rough inliers | num samples that had this many: " << endl;
    oss << printHistogram(prefix + "  ", num_rough_inliers_) << endl;
  }
  if(!num_refined_inliers_.empty()) {
    oss << prefix << "Histogram of num refined inliers | num samples that had this many: " << endl;
    oss << printHistogram(prefix + "  ", num_refined_inliers_) << endl;
  }
  if(!edge_match_pcts_.empty()) {
    oss << prefix << "Histogram of edge match pcts | num samples that had this many: " << endl;
    oss << printHistogram(prefix + "  ", edge_match_pcts_, 2) << endl;
  }

  return oss.str();
}

FeatureSet search(const clams::Frame& frame, const FeatureSet& model, const FeatureSet& image, vector<bool>* remaining)
{
  cout << "============================================================ New search" << endl;
  
  ROS_ASSERT(!model.keypoints_.empty());
  ROS_ASSERT(!image.keypoints_.empty());

  // Params.
  // TODO: These should elsewhere.  This function should probably be in its own object
  // and have a YAML for configuration.
  int k = 3;
  int max_consecutive_nondetections = 1e4;
  float inlier_distance_thresh = 0.03;  // cm
  int descriptor_distance_thresh = 60;
  double edge_match_pct_thresh = 0.75;
  size_t model_inliers_set_thresh = 15;
 
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

    //cout << "descriptor distance: " << match.distance << endl;
  }
  // cout << "Mean descriptor distance to match: " << total_descriptor_distance / num_matches << endl;
  // cout << "Min descriptor distance: " << min_descriptor_distance << endl;
  // cout << "Max descriptor distance: " << max_descriptor_distance << endl;
  
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
  if(matches.size() < 10)
    return FeatureSet();

  // Compute canny image for use in final matching stage.
  cv::Mat1b gray;
  cv::cvtColor(frame.img_, gray, CV_BGR2GRAY);
  cv::Mat1b canny = defaultCanny(gray);
  cv::GaussianBlur(canny, canny, cv::Size(11, 11), 1);  // TODO: This should be a Laplacian blur.
  cv::imshow("Blurred Canny", canny);
  
  SearchStats stats;
  int best_num_inliers = 0;
  int num_samples_without_detection = 0;
  vector<size_t> best_inlier_indices;
  set<size_t> best_model_inliers_set;
  FeatureSet best_detection;
  double best_edge_match_pct = 0;
  while(true) {
    // If we haven't found anything in a while, stop.
    //cout << "num_samples_without_detection: " << num_samples_without_detection << endl;
    ++num_samples_without_detection;
    if(num_samples_without_detection > max_consecutive_nondetections)
      break;
    ++stats.num_samples_;

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
    ++stats.num_not_too_close_;
    
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
    stats.num_rough_inliers_.push_back(num_rough_inliers);
    if(num_rough_inliers < 3)
      continue;
    
    // Transform the model into the image using the refined cloud.
    Eigen::Affine3f trans_refined = tfc2.getTransformation();
    pcl::transformPointCloud(*model.keycloud_, transformed_model_keycloud, trans_refined);

    // Get inliers after refinement.
    set<size_t> model_inliers_set;
    vector<size_t> inlier_indices;
    inlier_indices.reserve(model.keycloud_->size());
    FeatureSet detection;
    for(size_t i = 0; i < matches.size(); ++i) {
      const cv::DMatch& match = matches[i];
      Point mpt = transformed_model_keycloud[match.trainIdx];
      Point ipt = image.keycloud_->at(match.queryIdx);
      float dist = pcl::euclideanDistance(mpt, ipt);
      if(dist < inlier_distance_thresh) {
        model_inliers_set.insert(match.trainIdx);
        inlier_indices.push_back(match.queryIdx);        
        detection.keypoints_.push_back(image.keypoints_[match.queryIdx]);
      }
    }
    stats.num_refined_inliers_.push_back(model_inliers_set.size());
    if(model_inliers_set.size() < model_inliers_set_thresh)
      continue;

    // Project the model pointcloud into the current camera coordinate system.
    detection.pcd_ = Cloud::Ptr(new Cloud);
    pcl::transformPointCloud(*model.pcd_, *detection.pcd_, trans_refined);

    // Check that this matches up.
    clams::FrameProjector proj = defaultProjector(frame);
    clams::ProjectivePoint ppt;
    double num_edge_points = 0;
    double num_edge_matches = 0;
    for(size_t i = 0; i < detection.pcd_->size(); ++i) {
      proj.project(detection.pcd_->at(i), &ppt);
      if(ppt.u_ < 0 || ppt.u_ >= frame.img_.cols)
        continue;
      if(ppt.v_ < 0 || ppt.v_ >= frame.img_.rows)
        continue;

      if(detection.pcd_->at(i).r > 0) {
        ++num_edge_points;
        if(canny(ppt.v_, ppt.u_) > 0)
          ++num_edge_matches;
      }
    }
    double edge_match_pct = num_edge_matches / num_edge_points;
    stats.edge_match_pcts_.push_back(edge_match_pct);
    if(edge_match_pct < edge_match_pct_thresh)
      continue;
    
    // If it's the best, make a note.
    if(model_inliers_set.size() > best_model_inliers_set.size()) {
      best_model_inliers_set = model_inliers_set;
      best_edge_match_pct = edge_match_pct;
      best_inlier_indices = inlier_indices;
      best_detection = detection;
    }
  }

  cout << "SearchStats: " << endl;
  cout << stats.status("  ") << endl;

  cout << "best_model_inliers_set.size(): " << best_model_inliers_set.size() << endl;
  if(best_model_inliers_set.size() > model_inliers_set_thresh) {

    // The matches should come from roughly the same octave and orientation.
    // This is not at all the case though.  WTH?
    // for(size_t i = 0; i < best_detection.keypoints_.size(); ++i) {
    //   cout << "keypoint " << i << ": " << best_detection.keypoints_[i].angle << " " 
    //        << best_detection.keypoints_[i].octave << endl;
    // }
    
    remaining->clear();
    remaining->resize(image.keypoints_.size(), true);
    for(size_t i = 0; i < best_inlier_indices.size(); ++i)
      remaining->at(best_inlier_indices[i]) = false;

    cout << "Returning detection with edge match pct: " << best_edge_match_pct << endl;
    return best_detection;
  }
  
  cout << "No detections." << endl;
  return FeatureSet();  // No detection.
}

void search(const clams::Frame& frame, const FeatureSet& model, FeatureSet image, vector<FeatureSet>* detections)
{
  detections->clear();

  while(true) {
    // Find a match.
    vector<bool> remaining;
    FeatureSet detection = search(frame, model, image, &remaining);
    if(detection.keypoints_.empty())
      break;
    detections->push_back(detection);

    size_t num_remaining = 0;
    for(size_t i = 0; i < remaining.size(); ++i)
      if(remaining[i])
        ++num_remaining;

    // Remove the features that were used in this match.
    FeatureSet reduced;
    reduced.keycloud_ = Cloud::Ptr(new Cloud());
    reduced.keycloud_->reserve(num_remaining);
    reduced.keypoints_.reserve(num_remaining);
    reduced.descriptors_ = cv::Mat1b(num_remaining, image.descriptors_.cols);
    size_t idx = 0;
    for(size_t i = 0; i < remaining.size(); ++i) {
      if(remaining[i]) {
        reduced.keycloud_->push_back(image.keycloud_->at(i));
        reduced.keypoints_.push_back(image.keypoints_[i]);
        for(int j = 0; j < image.descriptors_.cols; ++j)
          reduced.descriptors_(idx, j) = image.descriptors_(i, j);
        ++idx;
      }
    }
    ROS_ASSERT(idx == num_remaining);
    image = reduced;
  }
}

void RodVisualizer::detect()
{
  if(model_.keypoints_.empty()) {
    cout << "You must select a model using 's' first." << endl;
    return;
  }

  // Compute features on the current frame.
  clams::Frame frame = getFrame();
  FeatureSet fs_image;
  computeFeatures(frame, &fs_image, 1000);

  // Search for the model.
  vector<FeatureSet> detections;
  search(frame, model_, fs_image, &detections);
  cout << "Total detections: " << detections.size() << endl;
  
  // Visualize detections.
  cv::Mat3b vis = frame.img_.clone();
  for(size_t i = 0; i < detections.size(); ++i)
    detections[i].drawCloud(vis);
  for(size_t i = 0; i < detections.size(); ++i)
    detections[i].drawKeypoints(vis);
  cv::imshow("Detections", vis);

  if(SAVE_IMGS)
    cv::imwrite(nextPath(".", "", "-detections.png", 4), vis);
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
  computeFeatures(frame, &fs, 1000);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  cv::Mat3b vis = frame.img_.clone();
  fs.drawKeypoints(vis);
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
