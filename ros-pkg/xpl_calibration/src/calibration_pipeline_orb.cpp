#include <xpl_calibration/calibration_pipeline_orb.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

CalibrationPipelineOrb::CalibrationPipelineOrb(int num_threads) :
  pl_(num_threads)
{
  initializePipeline();
}

Eigen::Affine3f CalibrationPipelineOrb::calibrate(rgbd::Sequence::ConstPtr seq0,
						  rgbd::Sequence::ConstPtr seq1)
{
  pl_.setInput("Sequence0", seq0);
  pl_.setInput("Sequence1", seq1);
  pl_.compute();
  return pl_.getOutput<Affine3f>("TransformValidator", "BestTransform");
}

void CalibrationPipelineOrb::initializePipeline()
{
  registerPods();

  EntryPoint<SequenceConstPtr>* ep0 = new EntryPoint<SequenceConstPtr>("Sequence0");
  EntryPoint<SequenceConstPtr>* ep1 = new EntryPoint<SequenceConstPtr>("Sequence1");

  FrameSelector* fs0 = new FrameSelector("FrameSelector0");
  FrameSelector* fs1 = new FrameSelector("FrameSelector1");
  fs0->setParam("FrameId", 0);
  fs1->setParam("FrameId", 0);
  fs0->registerInput("Sequence", ep0, "Output");
  fs1->registerInput("Sequence", ep1, "Output");

  OrbGenerator* og0 = new OrbGenerator("OrbGenerator0");
  OrbGenerator* og1 = new OrbGenerator("OrbGenerator1");
  og0->registerInput("Image", fs0, "Image");
  og1->registerInput("Image", fs1, "Image");
  
  OrbMatcher* om = new OrbMatcher("OrbMatcher");
  om->registerInput("Keypoints", og0, "Keypoints");
  om->registerInput("Keypoints", og1, "Keypoints");

  TransformValidator* tv = new TransformValidator("TransformValidator");
  tv->registerInput("Transforms", om, "Transforms");
  
  pl_.addConnectedComponent(ep0);
}

void CalibrationPipelineOrb::registerPods() const
{
  REGISTER_POD_TEMPLATE(EntryPoint, SequenceConstPtr);
  REGISTER_POD(FrameSelector);
  REGISTER_POD(OrbGenerator);
  REGISTER_POD(OrbMatcher);
  REGISTER_POD(TransformValidator);
}

