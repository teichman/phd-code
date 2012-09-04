#ifndef FRAME_ALIGNER_H
#define FRAME_ALIGNER_H

#include <rgbd_sequence/primesense_model.h>

class FrameAligner
{
public:
  FrameAligner(const rgbd::PrimeSenseModel& model0, const rgbd::PrimeSenseModel& model1);
  //! Returns transform that takes 1 to 0.
  Eigen::Affine3f align(rgbd::Frame frame0, rgbd::Frame frame1) const;

protected:
  rgbd::PrimeSenseModel model0_;
  rgbd::PrimeSenseModel model1_;
  Eigen::Affine3f f0_to_f1_;
};

#endif // FRAME_ALIGNER_H
