#ifndef PRIMESENSE_SLAM_H
#define PRIMESENSE_SLAM_H

#include <pcl/filters/voxel_grid.h>
#include <bag_of_tricks/agent.h>
#include <pose_graph_slam/pose_graph_slam.h>
#include <rgbd_sequence/stream_sequence.h>
#include <xpl_calibration/trajectory.h>
#include <xpl_calibration/frame_aligner.h>

class PrimeSenseSlam : public Agent
{
public:
  typedef boost::shared_ptr<cv::Mat1f> FeaturesPtr;
  typedef boost::shared_ptr<const cv::Mat1f> FeaturesConstPtr;

  // -- Inputs
  rgbd::StreamSequence::ConstPtr sseq_;
  
  // -- Params
  //! When choosing the next frame, advance by at least this much.
  double min_dt_;
  //! Used in generating the final saved map and in frame alignment.
  double max_range_;
  
  // -- Outputs
  Trajectory traj_;
  //! Just for visualization.
  rgbd::Cloud map_;
  
  // -- Methods
  PrimeSenseSlam();
  void _run();

protected:
  std::map< size_t, std::vector<cv::KeyPoint> > close_keypoint_cache_;
  std::map< size_t, std::vector<cv::KeyPoint> > all_keypoint_cache_;
  std::map< size_t, FeaturesPtr > close_feature_cache_;
  std::map< size_t, FeaturesPtr > all_feature_cache_;
  std::vector<size_t> cached_frames_;

  void buildMap(const Trajectory& traj);
  //! Fills caches.
  FeaturesPtr getFeatures(const rgbd::Frame &frame, size_t t, std::vector<cv::KeyPoint> &keypoints);
};

class SlamVisualizer : public Agent
{
public:
  PrimeSenseSlam* slam_;

  void _run();
};

#endif // PRIMESENSE_SLAM_H
