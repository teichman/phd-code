#include <jarvis/jarvis.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>
#include <bag_of_tricks/connected_components.h>

using namespace std;
using namespace Eigen;

Jarvis::Jarvis(int vis_level, int rotation, string output_directory, bool write_video_frames) :
  record_(false),
  min_predictions_(1),
  min_confidence_(0),
  tracker_(100),
  vis_level_(vis_level),
  write_video_frames_(write_video_frames),
  rotation_(rotation)
{
  if(output_directory != "") {
    tda_ = TrackDatasetAssembler::Ptr(new TrackDatasetAssembler(output_directory, 30, 150, 10000));
  }
  
  fg_sub_ = nh_.subscribe("foreground", 3, &Jarvis::foregroundCallback, this);
  bg_sub_ = nh_.subscribe("background", 3, &Jarvis::backgroundCallback, this);
  gc_sub_ = nh_.subscribe("grid_classifier", 3, &Jarvis::gridClassifierCallback, this);
  det_pub_ = nh_.advertise<jarvis::Detection>("detections", 0);
  
  if(vis_level_ > 1)
    tracker_.visualize_ = true;

  cv::namedWindow("tracks", cv::WINDOW_NORMAL);
}

void Jarvis::backgroundCallback(sentinel::BackgroundConstPtr msg)
{
  // reconstructor_.update(msg);
  // if(reconstructor_.img_.rows > 0) {
  //   cv::imshow("background", reconstructor_.img_);
  //   cv::waitKey(2);
  // }
}

void Jarvis::detect(sentinel::ForegroundConstPtr fgmsg)
{
  ROS_ASSERT(gc_ && dp_);
  scopeLockRead;

  // -- Classify all blobs, add to cumulative predictions, and send messages for each.
  map<size_t, Blob::Ptr>::const_iterator it;
  for(it = tracker_.tracks_.begin(); it != tracker_.tracks_.end(); ++it) {
    size_t id = it->first;
    Blob::Ptr blob = it->second;
    ROS_ASSERT(blob);
    // Ignore tracks that don't have an update for this frame.
    if(blob->sensor_timestamp_ != fgmsg->sensor_timestamp)
      continue;

    // Make frame prediction and add to the DBF.
    if(!blob->cloud_)
      blob->project(false);
    Label fpred = gc_->classify(*dp_->computeDescriptors(blob));
    filters_[id].addObservation(fpred, blob->centroid_, blob->sensor_timestamp_);

    // Construct message for this track.
    jarvis::Detection msg;
    msg.header.stamp = fgmsg->header.stamp;
    msg.sensor_timestamp = fgmsg->sensor_timestamp;
    msg.track_id = id;
    msg.centroid = eigen_extensions::eigToVec(blob->centroid_);
    msg.cmap = gc_->nameMapping("cmap").names();
    msg.frame_prediction = fpred.vector();
    msg.track_prediction = filters_[id].trackPrediction().vector();
    msg.num_frames = filters_[id].numObservations();

    // Get image coords.
    msg.upper_left.x = blob->width_;
    msg.upper_left.y = blob->height_;
    msg.lower_right.x = 0;
    msg.lower_right.y = 0;
    for(size_t i = 0; i < blob->indices_.size(); ++i) {
      uint16_t y = blob->indices_[i] / blob->width_;
      uint16_t x = blob->indices_[i] - blob->width_ * y;
      msg.upper_left.x = min(msg.upper_left.x, x);
      msg.upper_left.y = min(msg.upper_left.y, y);
      msg.lower_right.x = max(msg.lower_right.x, x);
      msg.lower_right.y = max(msg.lower_right.y, y);
    }

    det_pub_.publish(msg);
  }

  // -- Wipe out DBFs for any tracks that no longer exist.
  auto fit = filters_.begin();
  while(fit != filters_.end()) {
    if(!tracker_.tracks_.count(fit->first))
      filters_.erase(fit++);
    else
      fit++;
  }
}

void Jarvis::foregroundCallback(sentinel::ForegroundConstPtr msg)
{
  //reconstructor_.update(msg);

  // -- Update the message timer.
  //    This is used for automatically terminating jarvis if nothing is heard for a while.
  message_hrt_.reset();
  message_hrt_.start();
  
  // -- Run the tracker.
  tracker_.update(msg);

  // -- Classify blobs, accumulate predictions for each track, and send Detection messages.
  //    Assumes tracker_ has been updated.
  if(gc_ && dp_)
    detect(msg);

  // -- Accumulate TrackDatasets.
  if(tda_)
    tda_->update(tracker_.tracks_);


  if(vis_level_ > 0) {
    // -- Allocate memory if necessary.
    if(color_vis_.rows != msg->height) {
      color_vis_ = cv::Mat3b(cv::Size(msg->width, msg->height));
      depth_vis_ = cv::Mat3b(cv::Size(msg->width, msg->height));
    }
    color_vis_ = cv::Vec3b(127, 127, 127);
    depth_vis_ = cv::Vec3b(0, 0, 0);
    
    // -- Draw tracks.
    tracker_.draw(color_vis_, true, filters_);
    cv::Mat3b color_vis_scaled;
    cv::resize(color_vis_, color_vis_scaled, color_vis_.size() * 2, cv::INTER_NEAREST);
    orient(rotation_, &color_vis_scaled);
    addTimestamp(msg->header.stamp.toBoost(), color_vis_scaled);
    cv::imshow("tracks", color_vis_scaled);

    if(write_video_frames_) {
      ostringstream oss;
      oss << fixed << setprecision(16) << setw(16) << setfill('0')
          << msg->header.stamp.toSec() << ".jpg";
      cv::imwrite(oss.str(), color_vis_scaled);
    }
  }

  if(vis_level_ > 1) {
    // -- Draw a depth visualization.
    depth_vis_ = cv::Vec3b(0, 0, 0);
    for(size_t i = 0; i < msg->indices.size(); ++i) {
      uint32_t idx = msg->indices[i];
      int y = idx / depth_vis_.cols;
      int x = idx - y * depth_vis_.cols;
      depth_vis_(y, x) = colorize(msg->depth[i] * 0.001, 0, 5);
    }
    for(size_t i = 0; i < msg->fg_indices.size(); ++i) {
      uint32_t idx = msg->fg_indices[i];
      int y = idx / msg->width;
      int x = idx - y * msg->width;
      cv::circle(depth_vis_, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
    }
    for(size_t i = 0; i < msg->bg_fringe_indices.size(); ++i) {
      uint32_t idx = msg->bg_fringe_indices[i];
      int y = idx / msg->width;
      int x = idx - y * msg->width;
      cv::circle(depth_vis_, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
    }

    cv::Mat3b depth_vis_scaled;
    cv::resize(depth_vis_, depth_vis_scaled, depth_vis_.size() * 2, cv::INTER_NEAREST);
    orient(rotation_, &depth_vis_scaled);
    cv::imshow("depth", depth_vis_scaled);
  }

  if(vis_level_ > 0)
    cv::waitKey(2);
}

void Jarvis::gridClassifierCallback(blob::BinaryBlobConstPtr msg)
{
  GridClassifier::Ptr gc(new GridClassifier);
  blob::fromBinaryBlob(*msg, gc.get());
  ROS_DEBUG_STREAM("[Jarvis] Classifier received.");
  ROS_DEBUG_STREAM(endl << gc->status("    ", true));
  if(!gc_ || *gc != *gc_) {
    ROS_DEBUG_STREAM("[Jarvis] Updating classifier.");
    scopeLockWrite;
    gc_ = gc;
  }
}
