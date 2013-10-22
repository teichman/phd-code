#include <jarvis/jarvis.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>
#include <bag_of_tricks/connected_components.h>

using namespace std;
using namespace Eigen;

Jarvis::Jarvis(int vis_level, int rotation, string output_directory) :
  record_(false),
  min_predictions_(30),
  min_confidence_(1),
  tracker_(100),
  tda_(output_directory, 10, 100, 10000),
  vis_level_(vis_level),
  rotation_(rotation)
{
  fg_sub_ = nh_.subscribe("foreground", 3, &Jarvis::foregroundCallback, this);
  bg_sub_ = nh_.subscribe("background", 3, &Jarvis::backgroundCallback, this);
  det_pub_ = nh_.advertise<jarvis::Detection>("detections", 0);
  
  if(vis_level_ > 1)
    tracker_.visualize_ = true;
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

  // -- Classify all blobs and add to cumulative predictions.
  map<size_t, Blob::Ptr>::const_iterator it;
  for(it = tracker_.tracks_.begin(); it != tracker_.tracks_.end(); ++it) {
    size_t id = it->first;
    Blob::Ptr blob = it->second;
    ROS_ASSERT(blob);
    // Ignore tracks that don't have an update for this frame.
    if(blob->sensor_timestamp_ != fgmsg->sensor_timestamp)
      continue;
    predictions_[id].push_back(gc_->classify(*dp_->computeDescriptors(blob)));
  }
  
  // -- If any current blobs have passed threshold, send messages indicating their detection.
  cout << "============================================================" << endl;
  map<size_t, vector<Label> >::const_iterator pit;
  for(pit = predictions_.begin(); pit != predictions_.end(); ++pit) {
    size_t id = pit->first;
    // Ignore tracks that don't have an update for this frame.
    if(!tracker_.tracks_.count(id) || !tracker_.tracks_[id] || tracker_.tracks_[id]->sensor_timestamp_ != fgmsg->sensor_timestamp)
      continue;

    // Ignore tracks that haven't been seen for long enough.
    const vector<Label>& predictions = pit->second;
    cout << "Track " << id << " #predictions: " << predictions.size();
    if(predictions.size() < min_predictions_) {
      cout << endl;
      continue;
    }

    // Get the overall track prediction using a simple discrete Bayes filter.
    Label track_prediction = VectorXf::Zero(gc_->nameMapping("cmap").size());
    for(size_t i = 0; i < predictions.size(); ++i)
      track_prediction += predictions[i];
    track_prediction /= (float)predictions.size();
    track_prediction += gc_->prior();
    cout << " track_prediction: " << track_prediction.transpose() << endl;

    // If we have a positive detection of any of the classes, send a message.
    if((track_prediction.array() > min_confidence_).any()) {
      cout << "Publishing..." << endl;
      jarvis::Detection msg;
      msg.sensor_timestamp = fgmsg->sensor_timestamp;
      msg.cmap = gc_->nameMapping("cmap").names();
      msg.label = track_prediction.vector();
      det_pub_.publish(msg);
    }
  }
}

void Jarvis::foregroundCallback(sentinel::ForegroundConstPtr msg)
{
  //reconstructor_.update(msg);

  // -- Run the tracker.
  tracker_.update(msg);

  // -- Classify blobs, accumulate predictions for each track, and send Detection messages.
  //    Assumes tracker_ has been updated.
  if(gc_ && dp_)
    detect(msg);

  // -- Accumulate TrackDatasets.
  if(record_)
    tda_.update(tracker_.tracks_);


  if(vis_level_ > 0) {
    // -- Allocate memory if necessary.
    if(color_vis_.rows != msg->height) {
      color_vis_ = cv::Mat3b(cv::Size(msg->width, msg->height));
      depth_vis_ = cv::Mat3b(cv::Size(msg->width, msg->height));
    }
    color_vis_ = cv::Vec3b(127, 127, 127);
    depth_vis_ = cv::Vec3b(0, 0, 0);
    
    // -- Draw tracks.
    tracker_.draw(color_vis_);
    cv::Mat3b color_vis_scaled;
    cv::resize(color_vis_, color_vis_scaled, color_vis_.size() * 2, cv::INTER_NEAREST);
    orient(rotation_, &color_vis_scaled);
    addTimestamp(msg->header.stamp.toBoost(), color_vis_scaled);
    cv::imshow("tracks", color_vis_scaled);
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

