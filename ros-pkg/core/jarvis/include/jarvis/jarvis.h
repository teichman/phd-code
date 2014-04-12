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
#include <jarvis/discrete_bayes_filter.h>
#include <blob/blob.h>

//! SharedLockable's mutex is used to protect gc_.
class Jarvis : public SharedLockable
{
public:
  bool record_;
  
  //! If gc_ and dp_ are provided, they will be used to classify Blobs and publish Detection messages.
  GridClassifier::Ptr gc_;
  DescriptorPipeline::Ptr dp_;
  size_t min_predictions_;
  float min_confidence_;
  
  //! rotation is the angle in degrees and must be one of 0, 90, 180, or 270.
  Jarvis(int vis_level, int rotation,
         std::string output_directory = "",
         bool write_video_frames = false);

  // Save .td files almost immediately for testing purposes.
  void debugTDA();
  
  //! Saves any unsaved tracks in the TrackDatasetAssembler and clears the current contents.
  void flush() { if(tda_) tda_->flush(); }
  double secondsSinceLastMessage() const { return message_hrt_.getSeconds(); }
  
protected:
  ros::NodeHandle nh_;
  ros::Subscriber fg_sub_;
  ros::Subscriber bg_sub_;
  ros::Subscriber gc_sub_;
  ros::Publisher det_pub_;
  cv::Mat3b color_vis_;
  cv::Mat3b depth_vis_;
  Reconstructor reconstructor_;
  Tracker tracker_;
  TrackDatasetAssembler::Ptr tda_;
  int vis_level_;
  std::string output_directory_;
  bool write_video_frames_;
  int rotation_;
  //! track id, track predictions.
  std::map<size_t, DiscreteBayesFilter> filters_;
  HighResTimer message_hrt_;
  
  void foregroundCallback(sentinel::ForegroundConstPtr msg);
  void backgroundCallback(sentinel::BackgroundConstPtr msg);
  void gridClassifierCallback(blob::BinaryBlobConstPtr msg);
  void detect(sentinel::ForegroundConstPtr msg);
};


#endif // JARVIS_H
