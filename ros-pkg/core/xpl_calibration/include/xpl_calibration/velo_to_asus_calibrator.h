#ifndef VELO_TO_ASUS_CALIBRATOR_H
#define VELO_TO_ASUS_CALIBRATOR_H

#include <xpl_calibration/object_matching_calibrator.h>  // For generateTransform.
#include <rgbd_sequence/primesense_model.h>

//! Computes the loss for a given set of frames and reference pcds when twiddling
//! sync offset and transform between pcds and frames.
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

  void incrementLoss(rgbd::Frame frame, const rgbd::Cloud& pcd, double* val, double* count) const;
};

class VeloToAsusCalibration : public Serializable
{
public:
  //! Added to velodyne timestamps.
  double offset_;
  Eigen::Affine3f velo_to_asus_;

  VeloToAsusCalibration();
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  std::string status(const std::string& prefix = "") const;
};

class VeloToAsusCalibrator
{
public:
  rgbd::PrimeSenseModel model_;
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  GridSearchViewHandler* vis_;
  
  VeloToAsusCalibrator(const rgbd::PrimeSenseModel& model, GridSearchViewHandler* vis = NULL);
  VeloToAsusCalibration search() const;
  
protected:
  
};

#endif // VELO_TO_ASUS_CALIBRATOR_H
