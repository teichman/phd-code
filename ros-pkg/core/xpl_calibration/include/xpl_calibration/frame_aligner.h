#ifndef FRAME_ALIGNER_H
#define FRAME_ALIGNER_H

#include <xpl_calibration/mean_depth_error.h>


class FrameAligner
{
public:
  double max_range_;
  
  FrameAligner(const rgbd::PrimeSenseModel& model0,
	       const rgbd::PrimeSenseModel& model1,
	       double max_range,
	       GridSearchViewHandler* view_handler = NULL);

  //! Computes transform that takes points in 0 to points in 1. Starts with identity transform, has a wide search.
  //! Returns false if no transform found.
  bool align(rgbd::Frame frame0, rgbd::Frame frame1,
	     const std::vector<cv::Point2d>& keypoints0, const std::vector<cv::Point2d>& keypoints1,
	     bool consider_wide_search, Eigen::Affine3d* f0_to_f1) const;
  
  //! Computes transform that takes points in 0 to points in 1. Starts with identity transform, has a wide search.
  //! Returns false if no transform found.
  bool unconfidentAlign(rgbd::Frame frame0, rgbd::Frame frame1,
			const std::vector<cv::Point2d>& keypoints0, const std::vector<cv::Point2d>& keypoints1,
			Eigen::Affine3d* f0_to_f1) const;
  
  //! Computes transform that takes points in 0 to points in 1.  Starts with guess, has a narrow search.
  //! Returns false if no transform found.
  bool confidentAlign(rgbd::Frame frame0, rgbd::Frame frame1,
		      const std::vector<cv::Point2d>& keypoints0, const std::vector<cv::Point2d>& keypoints1, 
		      const Eigen::Affine3d& guess,
		      Eigen::Affine3d* f0_to_f1) const;

  //! Using keypoint matching method only to compute transform that takes points in 0 to points in 1.
  //! Returns false if no transform found.
  bool computeRoughTransform(const std::vector<cv::KeyPoint>& keypoints0, const std::vector<cv::KeyPoint>& keypoints1,
			     FeaturesConstPtr features0, FeaturesConstPtr features1,
			     const std::vector<cv::Point2d>* correspondences0, const std::vector<cv::Point2d>* correspondences1,
			     Eigen::Affine3d* f0_to_f1) const;
  
protected:
  rgbd::PrimeSenseModel model0_;
  rgbd::PrimeSenseModel model1_;
  Eigen::Affine3d f0_to_f1_;
  GridSearchViewHandler* view_handler_;
};

#endif // FRAME_ALIGNER_H
