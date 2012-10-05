#include <boost/program_options.hpp>
#include <xpl_calibration/primesense_slam.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
namespace bfs = boost::filesystem;

std::string sseqName(std::string str)
{
  if(str[str.size()-1] == '/')
    str = str.substr(0, str.size() - 1);
  bfs::path path(str);
  return path.filename();
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data.")
    ("frame0", bpo::value<int>()->required(), "")
    ("frame1", bpo::value<int>()->required(), "")
    ("consider-wide-search", "")
    ("max-range", bpo::value<double>())
    ("depth-weight", bpo::value<double>())
    ("color-weight", bpo::value<double>())
    ("keypoint-weight", bpo::value<double>())
    ("keypoint-hinge", bpo::value<double>())
    ("fraction", bpo::value<double>())
    ("num-ransac-samples", bpo::value<int>())
    ("min-ransac-inlier-percent", bpo::value<double>())
    ("keypoints-per-frame", bpo::value<int>())
    ("min-pairwise-keypoint-dist", bpo::value<double>())
    ("ransac-max-inlier-dist", bpo::value<double>())
    ("gridsearch-type", bpo::value<string>())
    ;

  p.add("sseq", 1).add("frame0", 1).add("frame1", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ FRAME0 FRAME1 [OPTS]" << endl;
    cout << "       To save the final alignment, press 'd' in PCLVis." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }
  
  cout << "Using " << opts["sseq"].as<string>() << endl;
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(opts["sseq"].as<string>());

  // -- Parse params.
  FrameAligner aligner(sseq->model_, sseq->model_);
  if(opts.count("max-range"))
    aligner.params_.set("max_range", opts["max-range"].as<double>());
  if(opts.count("depth-weight"))
    aligner.params_.set("depth_weight", opts["depth-weight"].as<double>());
  if(opts.count("color-weight"))
    aligner.params_.set("color_weight", opts["color-weight"].as<double>());
  if(opts.count("keypoint-weight"))
    aligner.params_.set("keypoint_weight", opts["keypoint-weight"].as<double>());
  if(opts.count("keypoint-hinge"))
    aligner.params_.set("keypoint_hinge", opts["keypoint-hinge"].as<double>());
  if(opts.count("fraction"))
    aligner.params_.set("fraction", opts["fraction"].as<double>());
  if(opts.count("num-ransac-samples"))
    aligner.params_.set("num_ransac_samples", opts["num-ransac-samples"].as<int>());
  if(opts.count("min-ransac-inlier-percent"))
    aligner.params_.set("min_ransac_inlier_percent", opts["min-ransac-inlier-percent"].as<double>());
  if(opts.count("min-pairwise-keypoint-dist"))
    aligner.params_.set("min_pairwise_keypoint_dist", opts["min-pairwise-keypoint-dist"].as<double>());
  if(opts.count("ransac-max-inlier-dist"))
    aligner.params_.set("ransac_max_inlier_dist", opts["ransac-max-inlier-dist"].as<double>());
  if(opts.count("gridsearch-type"))
    aligner.params_.set("gridsearch_type", opts["gridsearch-type"].as<string>());
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
  if(opts.count("keypoints-per-frame"))
    pss.keypoints_per_frame_ = opts["keypoints-per-frame"].as<int>();
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
  cout << "Transform: " << endl;
  cout << f0_to_f1.matrix() << endl;

  cout << "Save alignment? (y / n): ";
  string input;
  cin >> input;
  if(input == "y") {
    string alignments_dir = "alignments";
    if(!bfs::exists(alignments_dir))
      bfs::create_directory(alignments_dir);

    ostringstream oss;
    oss << alignments_dir << "/" << sseqName(opts["sseq"].as<string>()) << "-" << opts["frame0"].as<int>() << ":" << opts["frame1"].as<int>();
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
