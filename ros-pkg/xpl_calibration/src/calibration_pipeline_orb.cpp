#include <xpl_calibration/calibration_pipeline_orb.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
using namespace pipeline;

CalibrationPipelineOrb::CalibrationPipelineOrb(int num_threads, std::string pipeline_file) :
  pl_(num_threads)
{
  registerPods();
  if(pipeline_file.empty())
    initializePipeline();
  else
    pl_.load(pipeline_file);
}

Eigen::Affine3f CalibrationPipelineOrb::calibrate(rgbd::Sequence::ConstPtr seq0,
						  rgbd::Sequence::ConstPtr seq1)
{
  pl_.setInput("Sequence0", seq0);
  pl_.setInput("Sequence1", seq1);
  pl_.setDebug(true);
  pl_.compute();

  return *pl_.getOutput<const Affine3f*>("TransformValidator", "BestTransform");
}

void CalibrationPipelineOrb::initializePipeline()
{ 
  EntryPoint<SequenceConstPtr>* ep0 = new EntryPoint<SequenceConstPtr>("Sequence0");
  EntryPoint<SequenceConstPtr>* ep1 = new EntryPoint<SequenceConstPtr>("Sequence1");

  BackgroundModeler* bm0 = new BackgroundModeler("BackgroundModeler0");
  BackgroundModeler* bm1 = new BackgroundModeler("BackgroundModeler1");
  bm0->registerInput("Sequence", ep0, "Output");
  bm1->registerInput("Sequence", ep1, "Output");
  
  FrameSelector* fs0 = new FrameSelector("FrameSelector0");
  FrameSelector* fs1 = new FrameSelector("FrameSelector1");
  fs0->setParam("FrameId", 0);
  fs1->setParam("FrameId", 0);
  fs0->registerInput("Sequence", ep0, "Output");
  fs1->registerInput("Sequence", ep1, "Output");

  KdTreePod* kd0 = new KdTreePod("KdTreePod0");
  KdTreePod* kd1 = new KdTreePod("KdTreePod1");
  kd0->registerInput("Cloud", fs0, "Cloud");
  kd1->registerInput("Cloud", fs1, "Cloud");
  
  OrbExtractor* og0 = new OrbExtractor("OrbExtractor0");
  OrbExtractor* og1 = new OrbExtractor("OrbExtractor1");
  og0->registerInput("Image", fs0, "Image");
  og1->registerInput("Image", fs1, "Image");
  
  OrbMatcher* om = new OrbMatcher("OrbMatcher");
  om->registerInput("Keypoints0", og0, "Keypoints");
  om->registerInput("Keypoints1", og1, "Keypoints");
  om->registerInput("Descriptors0", og0, "Descriptors");
  om->registerInput("Descriptors1", og1, "Descriptors");
  om->registerInput("Image0", fs0, "Image");
  om->registerInput("Image1", fs1, "Image");
  om->registerInput("Cloud0", fs0, "Cloud");
  om->registerInput("Cloud1", fs1, "Cloud");

  TransformValidator* tv = new TransformValidator("TransformValidator");
  tv->registerInput("Candidates", om, "Transforms");
  tv->registerInput("Cloud0", fs0, "Cloud");
  tv->registerInput("Cloud1", fs1, "Cloud");
  tv->registerInput("KdTree0", kd0, "KdTree");
  tv->registerInput("KdTree1", kd1, "KdTree");
    
  pl_.addConnectedComponent(ep0);
  pl_.writeGraphviz("graphviz");
  pl_.save("calibration_pipeline_orb.pl");
}

void CalibrationPipelineOrb::registerPods() const
{
  REGISTER_POD_TEMPLATE(EntryPoint, SequenceConstPtr);
  REGISTER_POD(FrameSelector);
  REGISTER_POD(OrbExtractor);
  REGISTER_POD(OrbMatcher);
  REGISTER_POD(TransformValidator);
  REGISTER_POD(KdTreePod);
  REGISTER_POD(BackgroundModeler);
}

