#include <eigen_extensions/eigen_extensions.h>
#include <xpl_calibration/calibration_pipeline_orb.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;
using namespace rgbd;
using namespace pipeline;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

string usageString()
{
  ostringstream oss;
  oss << "Usage: visualize_background_subtraction SEQ [PIPELINE]" << endl;
  return oss.str();
}


class KBHandler
{
public:
  bool pressed_;

  KBHandler() :
    pressed_(false)
  {
  }
  
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event)
  {
    pressed_ = true;
    
    if (event.getKeyCode())
      cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
    else
      cout << "the special key \'" << event.getKeySym() << "\' was";
    if (event.keyDown())
      cout << " pressed" << endl;
    else
      cout << " released" << endl;  
  }
};

int main(int argc, char** argv)
{
  if(argc != 2 && argc != 3) {
    cout << usageString() << endl;
    return 0;
  }

  typedef Sequence::ConstPtr SequenceConstPtr;
  REGISTER_POD_TEMPLATE(EntryPoint, SequenceConstPtr);
  REGISTER_POD(GaussianBackgroundModeler);
  REGISTER_POD(HistogramBackgroundModeler);
  REGISTER_POD(BackgroundSubtractor);
  REGISTER_POD(ObjectExtractor);
  
  Pipeline pl(NUM_THREADS);
  if(argc == 3) { 
    string pipeline_path = argv[2];
    cout << "Using custom Pipeline specification: " << pipeline_path << endl;
    pl.load(pipeline_path);
  }
  else { 
    EntryPoint<Sequence::ConstPtr>* ep = new EntryPoint<Sequence::ConstPtr>("Sequence");
    HistogramBackgroundModeler* bm = new HistogramBackgroundModeler("HistogramBackgroundModeler");
    bm->registerInput("Sequence", ep, "Output");
    BackgroundSubtractor* bs = new BackgroundSubtractor("BackgroundSubtractor");
    bs->registerInput("Sequence", ep, "Output");
    bs->registerInput("BackgroundModel", bm, "BackgroundModel");
    ObjectExtractor* oe = new ObjectExtractor("ObjectExtractor");
    oe->registerInput("Sequence", ep, "Output");
    oe->registerInput("ForegroundImages", bs, "ForegroundImages");
    oe->registerInput("ForegroundIndices", bs, "ForegroundIndices");
    
    pl.addConnectedComponent(ep);
    pl.save("bgs-default.pl");
  }
  
  StreamSequence::Ptr sseq(new StreamSequence);
  cout << "Loading " << argv[1] << "." << endl;
  sseq->load(argv[1]);

  // -- Downsample the sequence heavily.
  size_t interval = 20;
  Sequence::Ptr seq(new Sequence);
  seq->pcds_.reserve(sseq->size());
  seq->imgs_.reserve(sseq->size());
  for(size_t i = 0; i < sseq->size(); i += interval) {
    seq->pcds_.push_back(sseq->getCloud(i));
    seq->imgs_.push_back(sseq->getImage(i));
  }

  // -- Run pipeline.
  pl.setDebug(true);
  pl.setInput<Sequence::ConstPtr>("Sequence", seq);
  pl.compute();
  const vector< vector<int> >* fg_indices;
  pl.pull("BackgroundSubtractor", "ForegroundIndices", &fg_indices);

  pcl::visualization::CloudViewer vis("Foreground");
  // KBHandler h;
  // vis.registerKeyboardCallback(boost::bind(&KBHandler::keyboardCallback, &h, _1));
  Cloud::Ptr fg(new Cloud);
  while(true) {    
    for(size_t i = 0; i < seq->size(); ++i) {
      const Cloud& pcd = *seq->pcds_[i];
      const vector<int>& indices = fg_indices->at(i);
      *fg = pcd;
      for(size_t j = 0; j < indices.size(); ++j) { 
	fg->at(indices[j]).r = 255;
	fg->at(indices[j]).g = 0;
	fg->at(indices[j]).b = 0;
      }
      
      vis.showCloud(fg);
      cin.ignore();
    }

    cout << "Press return to re-run." << endl;
    if(argc == 3) {
      pl.load(argv[2]);
      cout << "Re-loaded " << argv[2] << endl;

      pl.setInput<Sequence::ConstPtr>("Sequence", seq);
      pl.compute();
      pl.pull("BackgroundSubtractor", "ForegroundIndices", &fg_indices);
    }
  }
    
  return 0;
}
