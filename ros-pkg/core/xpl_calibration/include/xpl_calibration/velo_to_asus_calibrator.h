#ifndef VELO_TO_ASUS_CALIBRATOR_H
#define VELO_TO_ASUS_CALIBRATOR_H

#include <xpl_calibration/mean_depth_error.h>

class VeloToAsusCalibration : public Serializable
{
public:
  //! Added to velodyne timestamps to make asus & velo timestamps match.
  double offset_;

  const Eigen::Affine3f& veloToAsus() const { return velo_to_asus_; }
  const Eigen::Affine3f& asusToVelo() const { return asus_to_velo_; }
  void setVeloToAsus(const Eigen::Affine3f& transform) { velo_to_asus_ = transform; asus_to_velo_ = transform.inverse(); }
  
  VeloToAsusCalibration();
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  std::string status(const std::string& prefix = "") const;

protected:
  Eigen::Affine3f velo_to_asus_;
  Eigen::Affine3f asus_to_velo_;
};

class VeloToAsusCalibrator
{
public:
  rgbd::PrimeSenseModel model_;
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  GridSearchViewHandler* vis_;
  
  VeloToAsusCalibrator(const rgbd::PrimeSenseModel& model, GridSearchViewHandler* vis = NULL);
  VeloToAsusCalibration search(double* final_value = NULL) const;
  
protected:
  
};

#endif // VELO_TO_ASUS_CALIBRATOR_H
