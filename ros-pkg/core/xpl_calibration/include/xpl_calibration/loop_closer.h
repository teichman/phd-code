/*
 * =====================================================================================
 *
 *       Filename:  loop_closer.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/05/2012 01:14:54 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */
#ifndef LOOP_CLOSER_H
#define LOOP_CLOSER_H
#include <xpl_calibration/orb_extractor.h>
#include <xpl_calibration/orb_matcher.h>
#include <pose_graph_slam/pose_graph_slam.h>
#include <rgbd_sequence/stream_sequence.h>
#include <map>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <optimization/grid_search.h>

enum FeatureType
{
  SIFT,
  ORB,
  FPFH
};

enum VerificationType
{
  ICP,
  MDE
};

class LoopCloser : public GridSearchViewHandler
{
public:
  typedef boost::shared_ptr<LoopCloser> Ptr;
  typedef boost::shared_ptr<const LoopCloser> ConstPtr;
  typedef boost::shared_ptr<cv::Mat1f> FeaturesPtr;
  typedef boost::shared_ptr<const cv::Mat1f> FeaturesConstPtr;

  LoopCloser(rgbd::StreamSequence::ConstPtr sseq);
  bool getLinkHypotheses(const rgbd::Frame &frame, size_t t, vector<size_t> &targets, 
      vector<Eigen::Affine3f> &transforms);
  void handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective);

  //! Flags
  bool fine_tune_;
  bool visualize_;
  //! Loop closures must be at least this far apart in time
  int min_time_offset_;
  // Matching
  FeatureType ftype_;
  int keypoints_per_frame_;
  int k_;
  float max_feature_dist_;
  float max_z_; //z distance at which we throw out keypoints
  float fpfh_radius_;
  float harris_thresh_;
  size_t harris_margin_; //Amount to crop when finding 3D harris points
  bool use_3d_sift_;
  // RANSAC
  int num_ransac_samples_;
  float min_ransac_inlier_percent_;
  int min_ransac_inliers_;
  float min_pairwise_keypoint_dist_;
  float ransac_max_inlier_dist_;
  float min_bounding_length_;
  // ICP Verification
  VerificationType verification_type_;
  float icp_max_avg_dist_;
  float icp_inlier_percent_;
  float icp_max_inlier_dist_;
  // MDE Verification
  float max_mde_;
  
  //Visualization
  GridSearchViewHandler *view_handler_;
  pcl::visualization::PCLVisualizer vis_;
  rgbd::Cloud::Ptr cur_pcd_;
  rgbd::Cloud::Ptr cur_pcd_transformed_;
  rgbd::Cloud::Ptr prev_pcd_;

protected:
  //! If true, returns a set of target frame indices and transforms (cur -> prev), before fine tuning
  virtual bool getInitHypotheses(const rgbd::Frame &frame, size_t t, vector<size_t> &targets, 
      vector<Eigen::Affine3f> &transforms);
  //! Fine tune a hypothesis
  virtual Eigen::Affine3f fineTuneHypothesis(const rgbd::Frame &frame_cur, 
      const rgbd::Frame &frame_prev, const Eigen::Affine3f &hypothesis);
  //! Align frames
  virtual Eigen::Affine3f alignFrames(const rgbd::Frame &frame0, const rgbd::Frame &frame1, 
      const Eigen::Affine3f &guess);
  //! Get features from orb and cache them
  virtual FeaturesPtr getFeatures(const rgbd::Frame &frame, size_t t, vector<cv::KeyPoint> &keypoints);

  rgbd::StreamSequence::ConstPtr sseq_;
  //OrbExtractor oe_;
  //OrbMatcher om_;
  boost::shared_ptr<cv::ORB> orb_;
  boost::shared_ptr<cv::SIFT> sift_;

  std::map< size_t, vector<cv::KeyPoint> > keypoint_cache_;
  //std::map< size_t, OrbExtractor::PackedDescriptorsConstPtr > feature_cache_;
  std::map< size_t, FeaturesPtr > feature_cache_;
  std::vector<size_t> cached_frames_;
};

//class P2PDistance : public ScalarFunction
//{
//  public:
//    typedef boost::shared_ptr<P2PDistance> Ptr;
//    typedef boost::shared_ptr<const P2PDistance> ConstPtr;
//    //! Amount to downsample the 
//};

#endif //LOOP_CLOSER_H
