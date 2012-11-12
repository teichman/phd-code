#include <xpl_calibration/asus_vs_velo_visualizer.h>

class AVVModel
{
  StreamSequence::Ptr sseq_;
  VeloSequence::Ptr vseq_;
  VeloToAsusCalibration extrinsics_;
};

class AVVMultiviewModel
{
public:
  std::vector<AVVModel> models_;
  PrimeSenseModel learnDistortionModel() const;
};
