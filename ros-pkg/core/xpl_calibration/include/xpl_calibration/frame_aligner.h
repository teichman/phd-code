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
  //! Returns transform that takes points in 0 to points in 1.
  Eigen::Affine3d align(rgbd::Frame frame0, rgbd::Frame frame1,
			double* count, double* final_mde) const;
  
protected:
  rgbd::PrimeSenseModel model0_;
  rgbd::PrimeSenseModel model1_;
  Eigen::Affine3d f0_to_f1_;
  GridSearchViewHandler* view_handler_;
};

#endif // FRAME_ALIGNER_H
