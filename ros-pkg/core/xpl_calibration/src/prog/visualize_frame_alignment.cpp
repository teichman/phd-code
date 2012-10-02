#include <boost/program_options.hpp>
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
    ("max-range", bpo::value<double>()->default_value(10), "")
    ("depth-weight", bpo::value<double>()->default_value(1))
    ("color-weight", bpo::value<double>()->default_value(0.00115 / 2.0))
    ("keypoint-weight", bpo::value<double>()->default_value(0))
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
  
  FrameAligner aligner(sseq->model_, sseq->model_);
  aligner.params_.set("max_range", opts["max-range"].as<double>());
  cout << "FrameAligner is using params: " << endl;
  cout << aligner.params_ << endl;
  FrameAlignmentVisualizer fav(sseq->model_, sseq->model_);
  aligner.view_handler_ = &fav;
  Frame frame0, frame1;
  sseq->readFrame(opts["frame0"].as<int>(), &frame0);
  sseq->readFrame(opts["frame1"].as<int>(), &frame1);
  fav.setFrames(frame0, frame1);
    
  vector<cv::KeyPoint> keypoints0, keypoints1;
  PrimeSenseSlam pss;
  PrimeSenseSlam::FeaturesPtr features0 = pss.getFeatures(frame0, keypoints0);
  PrimeSenseSlam::FeaturesPtr features1 = pss.getFeatures(frame1, keypoints1);

  Affine3d f0_to_f1;
  // bool found = aligner.align(frame0, frame1,
  // 			     keypoints0, keypoints1,
  // 			     features0, features1,
  // 			     (bool)opts.count("consider-wide-search"), &f0_to_f1);
  ThreadPtr alignment_thread(new boost::thread(boost::bind(&FrameAligner::align, &aligner, frame0, frame1,
							   keypoints0, keypoints1,
							   features0, features1,
							   (bool)opts.count("consider-wide-search"), &f0_to_f1)));
  
  fav.run();
  alignment_thread->join();

  cout << "Save alignment? (y / n): ";
  string input;
  cin >> input;
  if(input == "y") {
    string alignments_dir = "alignments";
    if(!bfs::exists(alignments_dir))
      bfs::create_directory(alignments_dir);

    ostringstream oss;
    oss << alignments_dir << "/" << time(0);
    string alignment_dir = oss.str();
    cout << "Saving to " << alignment_dir << endl;
    bfs::create_directory(alignment_dir);
    frame0.save(alignment_dir + "/frame0");
    frame1.save(alignment_dir + "/frame1");
    eigen_extensions::saveASCII(f0_to_f1.matrix(), alignment_dir + "/f0_to_f1.eig.txt");
  }
  else
    cout << "Not saving." << endl;
  
  return 0;
}
