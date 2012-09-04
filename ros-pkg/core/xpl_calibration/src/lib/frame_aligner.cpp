#include <xpl_calibration/frame_aligner.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

FrameAligner::FrameAligner(const rgbd::PrimeSenseModel& model0,
			   const rgbd::PrimeSenseModel& model1) :
  model0_(model0),
  model1_(model1)
{
}

Eigen::Affine3f FrameAligner::align(rgbd::Frame frame0, rgbd::Frame frame1) const
{
  // MDE also checks alignment in time, but we don't want to use that functionality.
  frame0.timestamp_ = 0;
  frame1.timestamp_ = 0;

  MeanDepthError::Ptr mde(new MeanDepthError);
  
  Cloud::Ptr pcd0(new Cloud);
  model0_.frameToCloud(frame0, pcd0.get());
  Cloud::Ptr pcd1(new Cloud);
  model1_.frameToCloud(frame1, pcd1.get());
  
  mde.addFrame(frame0, model0_, pcd1);
  mde.addFrame(frame1, model1_, pcd0);
}

