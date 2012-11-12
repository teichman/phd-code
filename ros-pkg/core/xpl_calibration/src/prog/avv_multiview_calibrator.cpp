#include <boost/program_options.hpp>
#include <xpl_calibration/avv_multiview_model.h>

using namespace std;
using namespace pcl;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;

  vector<string> sseq_paths;
  vector<string> vseq_paths;
  vector<string> extrinsics_paths;
  
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseqs", bpo::value< vector<string> >(&sseq_paths)->required()->multitoken(), "")
    ("vseqs", bpo::value< vector<string> >(&vseq_paths)->required()->multitoken(), "")
    ("extrinsics", bpo::value< vector<string> >(&extrinsics_paths)->required()->multitoken(), "")
    ;

  bpo::positional_options_description p;
  p.add("sseq", 1).add("vseq", 1);
  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: avv_multiview_calibrator [OPTS] --sseqs SSEQ [SSEQ ...] --vseqs VSEQ [VSEQS] --extrinsics EXTRINSICS [EXTRINSICS ...]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ROS_ASSERT(sseq_paths.size() == vseq_paths.size());
  ROS_ASSERT(sseq_paths.size() == extrinsics_paths.size());
  cout << "==================================================" << endl;
  cout << "Using data:" << endl;
  cout << endl;
  for(size_t i = 0; i < sseq_paths.size(); ++i) {
    cout << sseq_paths[i] << endl;
    cout << vseq_paths[i] << endl;
    cout << extrinsics_paths[i] << endl;
    cout << endl;
  }
  cout << "==================================================" << endl;

  AVVMultiviewModel mvm;
  for(size_t i = 0; i < sseq_paths.size(); ++i) {
    AVVModel model;
    model.sseq_ = StreamSequence::Ptr(new StreamSequence);
    model.sseq_->load(sseq_paths[i]);
    model.vseq_ = VeloSequence::Ptr(new VeloSequence);
    model.vseq_->load(vseq_paths[i]);
    model.extrinsics_.load(extrinsics_paths[i]);
    mvm.models_.push_back(model);
  }

  cout << "Learning distortion models." << endl;
  PrimeSenseModel psm = mvm.learnDistortionModel();
  
  
  return 0;
}
