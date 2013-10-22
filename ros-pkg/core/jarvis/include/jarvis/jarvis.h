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

class Jarvis
{
public:
  //! If gc_ and dp_ are provided, they will be used to classify Blobs and publish Detection messages.
  GridClassifier::Ptr gc_;
  DescriptorPipeline::Ptr dp_;
  size_t min_predictions_;
  float min_confidence_;
  
  //! rotation is the angle in degrees and must be one of 0, 90, 180, or 270.
  Jarvis(int vis_level, int rotation, std::string output_directory);
  //! Saves any unsaved tracks in the TrackDatasetAssembler and clears the current contents.
  void flush() { tda_.flush(); }
  
protected:
  ros::NodeHandle nh_;
  ros::Subscriber fg_sub_;
  ros::Subscriber bg_sub_;
  ros::Publisher det_pub_;
  cv::Mat3b color_vis_;
  cv::Mat3b depth_vis_;
  Reconstructor reconstructor_;
  Tracker tracker_;
  TrackDatasetAssembler tda_;
  int vis_level_;
  int rotation_;
  //! track id, class predictions.  Matches tracker_.tracks_.
  std::map<size_t, std::vector<Label> > predictions_;
  
  void foregroundCallback(sentinel::ForegroundConstPtr msg);
  void backgroundCallback(sentinel::BackgroundConstPtr msg);
  void detect(sentinel::ForegroundConstPtr msg);
};

#endif // JARVIS_H
