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

FeatureSet search(const FeatureSet& model, const FeatureSet& image, vector<bool>* remaining)
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
      remaining->clear();
      remaining->resize(image.keypoints_.size(), true);
      for(size_t i = 0; i < inlier_indices.size(); ++i)
        remaining->at(inlier_indices[i]) = false;
      return detection;
    }
  }

  return FeatureSet();  // No detection.
}

void search(const FeatureSet& model, FeatureSet image, vector<FeatureSet>* detections)
{
  detections->clear();

  while(true) {
    // Find a match.
    vector<bool> remaining;
    FeatureSet detection = search(model, image, &remaining);
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
  computeFeatures(frame, &fs_image);

  // Search for the model.
  vector<FeatureSet> detections;
  search(model_, fs_image, &detections);
  
  // Visualize detections.
  cv::Mat3b vis = frame.img_.clone();
  for(size_t i = 0; i < detections.size(); ++i) { 
    detections[i].draw(vis);
  }
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
