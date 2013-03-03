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

Eigen::Affine3f CalibrationPipelineOrb::calibrate(rgbd::StreamSequence::ConstPtr sseq0,
                                                  rgbd::StreamSequence::ConstPtr sseq1)
{
  // -- Downsample the sequences.
  size_t interval = 20;
  double thresh = 0.3;
  Sequence::Ptr seq0(new Sequence);
  Sequence::Ptr seq1(new Sequence);
  seq0->pcds_.reserve(sseq0->size());
  seq1->pcds_.reserve(sseq1->size());
  seq0->imgs_.reserve(sseq0->size());
  seq1->imgs_.reserve(sseq1->size());
  Frame frame;
  for(size_t i = 0; i < sseq0->size(); i += interval) {
    double dt = -1;
    double ts0 = sseq0->timestamps_[i];
    sseq1->readFrame(ts0, &dt, &frame);
    if(fabs(dt) > thresh)
      continue;
    Cloud::Ptr pcd1(new Cloud);
    sseq1->model_.frameToCloud(frame, pcd1.get());
    seq1->imgs_.push_back(frame.img_);
    seq1->pcds_.push_back(pcd1);

    sseq0->readFrame(i, &frame);
    Cloud::Ptr pcd0(new Cloud);
    sseq0->model_.frameToCloud(frame, pcd0.get());

    seq0->pcds_.push_back(pcd0);
    seq0->imgs_.push_back(frame.img_);
  }

  // -- Compute calibration.
  pl_.setInput<Sequence::ConstPtr>("Sequence0", seq0);
  pl_.setInput<Sequence::ConstPtr>("Sequence1", seq1);
  pl_.setDebug(true);
  pl_.compute();

  return *pl_.getOutput<const Affine3f*>("TransformValidator", "BestTransform");
}

void CalibrationPipelineOrb::initializePipeline()
{ 
  EntryPoint<SequenceConstPtr>* ep0 = new EntryPoint<SequenceConstPtr>("Sequence0");
  EntryPoint<SequenceConstPtr>* ep1 = new EntryPoint<SequenceConstPtr>("Sequence1");

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
  REGISTER_POD(HistogramBackgroundModeler);
  REGISTER_POD(GaussianBackgroundModeler);
}

