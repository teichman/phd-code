#ifndef JARVIS_H
#define JARVIS_H

#include <iostream>
#include <deque>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <timer/timer.h>
#include <sentinel/reconstructor.h>
#include <online_learning/grid_classifier.h>
#include <jarvis/Detection.h>
#include <jarvis/track_dataset_assembler.h>
#include <jarvis/descriptor_pipeline.h>

class DiscreteBayesFilter
{
public:
  DiscreteBayesFilter(float cap = 30, double weight = 10, Label prior = Label());
  void addObservation(Label frame_prediction, const Eigen::VectorXf& centroid, double timestamp);
  Label trackPrediction() const;
  double timestamp() const { return prev_sensor_timestamp_; }
  size_t numObservations() const { return frame_predictions_.size(); }

protected:
  float cap_;
  //! Each log odds vector is weighted by dcentroid * weight.
  //! i.e. if the thing isn't moving, assume that we're not getting new information.
  double weight_;
  Label prior_;
  std::vector<Label> frame_predictions_;
  Label cumulative_;
  Eigen::VectorXf prev_centroid_;
  double prev_sensor_timestamp_;
};

class Jarvis
{
public:
  bool record_;
  
  //! If gc_ and dp_ are provided, they will be used to classify Blobs and publish Detection messages.
  GridClassifier::Ptr gc_;
  DescriptorPipeline::Ptr dp_;
  size_t min_predictions_;
  float min_confidence_;
  
  //! rotation is the angle in degrees and must be one of 0, 90, 180, or 270.
  Jarvis(int vis_level, int rotation, std::string output_directory = "");
  //! Saves any unsaved tracks in the TrackDatasetAssembler and clears the current contents.
  void flush() { if(tda_) tda_->flush(); }
  
protected:
  ros::NodeHandle nh_;
  ros::Subscriber fg_sub_;
  ros::Subscriber bg_sub_;
  ros::Publisher det_pub_;
  cv::Mat3b color_vis_;
  cv::Mat3b depth_vis_;
  Reconstructor reconstructor_;
  Tracker tracker_;
  TrackDatasetAssembler::Ptr tda_;
  int vis_level_;
  int rotation_;
  //! track id, track predictions.
  std::map<size_t, DiscreteBayesFilter> filters_;
  
  void foregroundCallback(sentinel::ForegroundConstPtr msg);
  void backgroundCallback(sentinel::BackgroundConstPtr msg);
  void detect(sentinel::ForegroundConstPtr msg);
};


#endif // JARVIS_H
