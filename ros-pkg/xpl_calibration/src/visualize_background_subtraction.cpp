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

int main(int argc, char** argv)
{
  if(argc != 2 && argc != 3) {
    cout << usageString() << endl;
    return 0;
  }

  typedef Sequence::ConstPtr SequenceConstPtr;
  REGISTER_POD_TEMPLATE(EntryPoint, SequenceConstPtr);
  REGISTER_POD(GaussianBackgroundModeler);
  REGISTER_POD(BackgroundModeler);
  REGISTER_POD(BackgroundSubtractor);
  
  Pipeline pl(NUM_THREADS);
  if(argc == 3) { 
    string pipeline_path = argv[2];
    cout << "Using custom Pipeline specification: " << pipeline_path << endl;
    pl.load(pipeline_path);
  }
  else { 
    EntryPoint<Sequence::ConstPtr>* ep = new EntryPoint<Sequence::ConstPtr>("Sequence");
    GaussianBackgroundModeler* bm = new GaussianBackgroundModeler("GaussianBackgroundModeler");
    //BackgroundModeler* bm = new BackgroundModeler("BackgroundModeler");
    bm->registerInput("Sequence", ep, "Output");
    BackgroundSubtractor* bs = new BackgroundSubtractor("BackgroundSubtractor");
    bs->registerInput("Sequence", ep, "Output");
    bs->registerInput("MaxDistances", bm, "MaxDistances");
    bs->registerInput("MinDistances", bm, "MinDistances");
    pl.addConnectedComponent(ep);
    pl.save("bgs-default.pl");
  }
  
  Sequence::Ptr seq(new Sequence);
  cout << "Loading " << argv[1] << "." << endl;
  seq->load(argv[1]);

  pl.setInput<Sequence::ConstPtr>("Sequence", seq);
  pl.compute();
  const vector< vector<int> >* fg_indices;
  pl.getOutput("BackgroundSubtractor", "ForegroundIndices", &fg_indices);

  pcl::visualization::CloudViewer vis("Foreground");
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
      usleep(1000 * 30);
    }

    cout << "Press return to re-run." << endl;
    cin.ignore();
    if(argc == 3) {
      pl.load(argv[2]);
      cout << "Re-loaded " << argv[2] << endl;

      pl.setInput<Sequence::ConstPtr>("Sequence", seq);
      pl.compute();
      pl.getOutput("BackgroundSubtractor", "ForegroundIndices", &fg_indices);
    }
  }
    
  return 0;
}