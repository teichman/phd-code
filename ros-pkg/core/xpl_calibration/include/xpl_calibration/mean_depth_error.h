#ifndef MEAN_DEPTH_ERROR_H
#define MEAN_DEPTH_ERROR_H

#include <xpl_calibration/object_matching_calibrator.h>  // For generateTransform.
#include <rgbd_sequence/primesense_model.h>

//! Computes the symmetric MDE between two nearby frames when twiddling the transform between the two.
class FrameAlignmentMDE : public ScalarFunction
{
public:
  FrameAlignmentMDE(const rgbd::PrimeSenseModel& model0, rgbd::Frame frame0, 
		    const rgbd::PrimeSenseModel& model1, rgbd::Frame frame1);
  //! x = [x, y, z, roll, pitch, yaw].
  double eval(const Eigen::VectorXd& x) const;
  
protected:
  rgbd::PrimeSenseModel model0_;
  rgbd::PrimeSenseModel model1_;
  rgbd::Frame frame0_;
  rgbd::Frame frame1_;
  rgbd::Cloud pcd0_;
  rgbd::Cloud pcd1_;
};

//! Computes the asymmetric MDE for a given set of frames and reference pcds when twiddling
//! sync offset and transform between pcds and frames.  It is asymmetric because we do not
//! assume the presence of a projection model for the reference PCDs.
class MeanDepthError : public ScalarFunction
{
public:
  //! Frames are from the PrimeSense, pcds are from the Velodyne.
  MeanDepthError(const rgbd::PrimeSenseModel& model,
		 const std::vector<rgbd::Frame>& frames,
		 const std::vector<rgbd::Cloud::ConstPtr>& pcds);
  //! x = [sync, x, y, z, roll, pitch, yaw].
  double eval(const Eigen::VectorXd& x) const;
  
protected:
  rgbd::PrimeSenseModel model_;
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  //! The maximum time difference to allow when choosing frame to pcd correspondences.
  double dt_thresh_;
};


void meanDepthError(const rgbd::PrimeSenseModel& model,
		    rgbd::Frame frame, const rgbd::Cloud& pcd,
		    double* count, double* val);

#endif // MEAN_DEPTH_ERROR_H
