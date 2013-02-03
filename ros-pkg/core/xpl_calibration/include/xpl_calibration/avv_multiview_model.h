#include <xpl_calibration/asus_vs_velo_visualizer.h>
#include <xpl_calibration/discrete_depth_distortion_model.h>

class AVVSequence
{
public:
  rgbd::StreamSequence::Ptr sseq_;
  VeloSequence::Ptr vseq_;
  VeloToAsusCalibration extrinsics_;
};

class AVVMultiviewModel
{
public:
  size_t step_;
  std::vector<AVVSequence> sequences_;

  AVVMultiviewModel();
  rgbd::PrimeSenseModel learnDistortionModel() const;
  DiscreteDepthDistortionModel learnDiscreteDistortionModel() const;

protected:
  bool veloYawValid(double yaw) const;
  rgbd::Cloud::Ptr filterVelo(const VeloToAsusCalibration& extrinsics,
                              rgbd::Cloud::ConstPtr velo) const;
};
