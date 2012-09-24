#ifndef FRAME_ALIGNER_H
#define FRAME_ALIGNER_H

#include <xpl_calibration/mean_depth_error.h>


class FrameAligner
{
public:
  typedef boost::shared_ptr<cv::Mat1f> FeaturesPtr;
  typedef boost::shared_ptr<const cv::Mat1f> FeaturesConstPtr;
  
  // -- Params for feature matching method
  int num_ransac_samples_;
  //! For knn.
  int k_;
  //! Arbitrary units
  float max_feature_dist_;
  float min_ransac_inliers_;
  //! meters
  float min_pairwise_keypoint_dist_;
  //! meters
  float ransac_max_inlier_dist_;
  //! [0, 1]
  float min_ransac_inlier_percent_;
  //! meters
  float min_bounding_length_;
  
  // -- Other params  
  double max_range_;
  
  FrameAligner(const rgbd::PrimeSenseModel& model0,
	       const rgbd::PrimeSenseModel& model1,
	       double max_range,
	       GridSearchViewHandler* view_handler = NULL);

  //! Computes transform that takes points in 0 to points in 1. Tries using feature matching to get close, and, failing that, does
  //! a wide area grid search if you tell it to.
  //! keypoints don't necessarily correspond.  Correspondences will be computed.
  //! Returns false if no transform found.
  bool align(rgbd::Frame frame0, rgbd::Frame frame1,
	     const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
	     FeaturesConstPtr features0, FeaturesConstPtr features1,
	     bool consider_wide_search, Eigen::Affine3d* f0_to_f1) const;
  
  //! Computes transform that takes points in 0 to points in 1. Starts with identity transform, has a wide search.
  //! Returns false if no transform found.
  bool unconfidentAlign(rgbd::Frame frame0, rgbd::Frame frame1,
			const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1,
			Eigen::Affine3d* f0_to_f1) const;
  
  //! Computes transform that takes points in 0 to points in 1.  Starts with guess, has a narrow search.
  //! Returns false if no transform found.
  bool confidentAlign(rgbd::Frame frame0, rgbd::Frame frame1,
		      const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1, 
		      const Eigen::Affine3d& guess,
		      Eigen::Affine3d* f0_to_f1) const;

  //! Using keypoint matching method only to compute transform that takes points in 0 to points in 1.
  //! correspondences will be filled with whatever can be found.
  //! Returns false if no transform found.
  bool computeRoughTransform(rgbd::Frame frame0, rgbd::Frame frame1,
			     const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
			     FeaturesConstPtr features0, FeaturesConstPtr features1,
			     std::vector<cv::Point2d>* correspondences0, std::vector<cv::Point2d>* correspondences1,
			     Eigen::Affine3d* f0_to_f1) const;
  
protected:
  rgbd::PrimeSenseModel model0_;
  rgbd::PrimeSenseModel model1_;
  GridSearchViewHandler* view_handler_;
};

#endif // FRAME_ALIGNER_H
