#include <xpl_calibration/calibration_pipeline_dynamic.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
using namespace pipeline;

#define ON (getenv("ON") ? atoi(getenv("ON")) : 5)
#define OFF (getenv("OFF") ? atoi(getenv("OFF")) : 50)
#define DEBUG (getenv("DEBUG") ? atoi(getenv("DEBUG")) : 0)

CalibrationPipelineDynamic::CalibrationPipelineDynamic(int num_threads, std::string pipeline_file) :
  pl_(num_threads)
{
  registerPods();
  if(pipeline_file.empty())
    initializePipeline();
  else
    pl_.load(pipeline_file);
}

void CalibrationPipelineDynamic::calibrate(rgbd::StreamSequence::ConstPtr sseq0,
					   rgbd::StreamSequence::ConstPtr sseq1,
					   Eigen::Affine3f* transform,
					   double* sync)
{
  // -- Downsample the sequences.
  double thresh = 0.05;
  Sequence::Ptr seq0(new Sequence);
  Sequence::Ptr seq1(new Sequence);
  seq0->pcds_.reserve(sseq0->size());
  seq1->pcds_.reserve(sseq1->size());
  seq0->imgs_.reserve(sseq0->size());
  seq1->imgs_.reserve(sseq1->size());
  pcl::visualization::CloudViewer vis("Matched");
  ROS_ASSERT(sseq0->size() == sseq0->timestamps_.size());
  bool on = false;
  for(size_t i = 0; i < sseq0->size(); ++i) {
    if(on && ((int)i % ON == 0))
      on = false;
    if(!on && ((int)i % OFF == 0))
      on = true;
    if(!on)
      continue;
    
    cout << "Considering frame " << i << endl;
    
    double dt = -1;
    double ts0 = sseq0->timestamps_[i];
    Cloud::Ptr pcd1 = sseq1->getCloud(ts0, &dt);
    if(dt > thresh)
      continue;


    vis.showCloud(pcd1);
    usleep(1000 * 30);
    seq0->pcds_.push_back(sseq0->getCloud(i));
    seq1->pcds_.push_back(pcd1);
    seq0->imgs_.push_back(sseq0->getImage(i));
    seq1->imgs_.push_back(sseq1->getImage(ts0, &dt));
    cout << "Added clouds with dt = " << dt << endl;
  }

  // -- Compute calibration.
  pl_.setInput<Sequence::ConstPtr>("Sequence0", seq0);
  pl_.setInput<Sequence::ConstPtr>("Sequence1", seq1);
  pl_.setDebug(DEBUG);
  pl_.compute();


  *transform = *pl_.getOutput<const Affine3f*>("ObjectMatchingCalibrator", "IcpRefinedTransform");
  //*transform = *pl_.getOutput<const Affine3f*>("ObjectMatchingCalibrator", "RansacRefinedTransform");
  *sync = pl_.getOutput<double>("ObjectMatchingCalibrator", "SyncOffset");
  cout << "Got sync offset of " << *sync << endl;
  cout << "transform: " << endl << transform->matrix() << endl;
}

void CalibrationPipelineDynamic::initializePipeline()
{ 
  EntryPoint<SequenceConstPtr>* ep0 = new EntryPoint<SequenceConstPtr>("Sequence0");
  EntryPoint<SequenceConstPtr>* ep1 = new EntryPoint<SequenceConstPtr>("Sequence1");

  HistogramBackgroundModeler* bm0 = new HistogramBackgroundModeler("HistogramBackgroundModeler0");
  HistogramBackgroundModeler* bm1 = new HistogramBackgroundModeler("HistogramBackgroundModeler1");
  bm0->registerInput("Sequence", ep0, "Output");
  bm1->registerInput("Sequence", ep1, "Output");

  BackgroundSubtractor* bs0 = new BackgroundSubtractor("BackgroundSubtractor0");
  BackgroundSubtractor* bs1 = new BackgroundSubtractor("BackgroundSubtractor1");
  bs0->registerInput("Sequence", ep0, "Output");
  bs0->registerInput("BackgroundModel", bm0, "BackgroundModel");
  bs1->registerInput("Sequence", ep1, "Output");
  bs1->registerInput("BackgroundModel", bm1, "BackgroundModel");

  ObjectExtractor* oe0 = new ObjectExtractor("ObjectExtractor0");
  ObjectExtractor* oe1 = new ObjectExtractor("ObjectExtractor1");
  oe0->registerInput("Sequence", ep0, "Output");
  oe0->registerInput("ForegroundImages", bs0, "ForegroundImages");
  oe0->registerInput("ForegroundIndices", bs0, "ForegroundIndices");
  oe1->registerInput("Sequence", ep1, "Output");
  oe1->registerInput("ForegroundImages", bs1, "ForegroundImages");
  oe1->registerInput("ForegroundIndices", bs1, "ForegroundIndices");

  ObjectMatchingCalibrator* omc = new ObjectMatchingCalibrator("ObjectMatchingCalibrator");
  omc->registerInput("Sequence0", ep0, "Output");
  omc->registerInput("Sequence1", ep1, "Output");
  omc->registerInput("Objects0", oe0, "Objects");
  omc->registerInput("Objects1", oe1, "Objects");
  
  // FrameSelector* fs0 = new FrameSelector("FrameSelector0");
  // FrameSelector* fs1 = new FrameSelector("FrameSelector1");
  // fs0->setParam("FrameId", 0);
  // fs1->setParam("FrameId", 0);
  // fs0->registerInput("Sequence", ep0, "Output");
  // fs1->registerInput("Sequence", ep1, "Output");

  // KdTreePod* kd0 = new KdTreePod("KdTreePod0");
  // KdTreePod* kd1 = new KdTreePod("KdTreePod1");
  // kd0->registerInput("Cloud", fs0, "Cloud");
  // kd1->registerInput("Cloud", fs1, "Cloud");

  // TransformValidator* tv = new TransformValidator("TransformValidator");
  // tv->registerInput("Candidates", om, "Transforms");
  // tv->registerInput("Cloud0", fs0, "Cloud");
  // tv->registerInput("Cloud1", fs1, "Cloud");
  // tv->registerInput("KdTree0", kd0, "KdTree");
  // tv->registerInput("KdTree1", kd1, "KdTree");
    
  pl_.addConnectedComponent(ep0);
  pl_.writeGraphviz("graphviz");
  pl_.save("calibration_pipeline_dynamic.pl");
}

void CalibrationPipelineDynamic::registerPods() const
{
  REGISTER_POD_TEMPLATE(EntryPoint, SequenceConstPtr);
  REGISTER_POD(FrameSelector);
  REGISTER_POD(TransformValidator);
  REGISTER_POD(KdTreePod);
  REGISTER_POD(HistogramBackgroundModeler);
  REGISTER_POD(BackgroundSubtractor);
  REGISTER_POD(ObjectExtractor);
  REGISTER_POD(ObjectMatchingCalibrator);
}

