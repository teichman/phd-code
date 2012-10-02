#include <xpl_calibration/primesense_slam.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data.")
    ("frame0", bpo::value<int>()->required(), "")
    ("frame1", bpo::value<int>()->required(), "")
    ("consider-wide-search", "")
    ;

  p.add("sseq", 1).add("frame0", 1).add("frame1", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ FRAME0 FRAME1 [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Using " << opts["sseq"].as<string>() << endl;
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(opts["sseq"].as<string>());
  
  FrameAligner aligner(sseq->model_, sseq->model_, 10.0);
  FrameAlignmentVisualizer fav(sseq->model_, sseq->model_);
  aligner.view_handler_ = &fav_;
  Frame frame0, frame1;

  vector<cv::KeyPoint> keypoints0, keypoints1;
  PrimeSenseSlam::FeaturesPtr features0 = PrimeSenseSlam::getFeatures(frame0, &keypoints0);
  PrimeSenseSlam::FeaturesPtr features1 = PrimeSenseSlam::getFeatures(frame1, &keypoints1);
  
  Affine3d f1_to_f0;
  bool found = aligner.align(frame1, frame0,
			     keypoints1, keypoints0,
			     features1, features0,
			     (bool)opts.count("consider-wide-search"), &f1_to_f0);
  cout << "Alignment considered successful: " << found << endl;
  
  return 0;
}
