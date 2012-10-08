#include <boost/program_options.hpp>
#include <xpl_calibration/primesense_slam.h>
//#define VISUALIZE
using namespace std;
using namespace Eigen;
using namespace rgbd;

class AlignmentEvaluation
{
public:
  double num_alignments_;
  double num_guessed_failed_;
  double total_translation_error_;
  double total_rotation_error_;
  double total_seconds_;
#ifdef VISUALIZE
  pcl::visualization::PCLVisualizer vis_; 
#endif

  AlignmentEvaluation() :
    num_alignments_(0),
    num_guessed_failed_(0),
    total_translation_error_(0),
    total_rotation_error_(0),
    total_seconds_(0)
  {
  }

  std::string status() const
  {
    ostringstream oss;
    oss << "num_alignments_: " << num_alignments_ << endl;
    oss << "num_guessed_failed_: " << num_guessed_failed_ << endl;
    oss << "mean translation error:" << total_translation_error_ / num_alignments_ << endl;
    oss << "mean rotation error: " << total_rotation_error_ / num_alignments_ << endl;
    oss << "mean alignment time: " << total_seconds_ / num_alignments_ << " seconds." << endl;
    return oss.str();
  }
};

void evaluate(string path, PrimeSenseModel model, AlignmentEvaluation* eval)
{
  Frame frame0, frame1;
  frame0.load(path + "/frame0");
  frame1.load(path + "/frame1");
  Matrix4d mat;
  eigen_extensions::loadASCII(path + "/f0_to_f1.eig.txt", &mat);

  FrameAligner aligner(model, model);
  vector<cv::KeyPoint> keypoints0, keypoints1;
  PrimeSenseSlam pss;
  PrimeSenseSlam::FeaturesPtr features0 = pss.getFeatures(frame0, keypoints0);
  PrimeSenseSlam::FeaturesPtr features1 = pss.getFeatures(frame1, keypoints1);
  Affine3d f0_to_f1;
  HighResTimer hrt("alignment");
  hrt.start();
  bool found = aligner.align(frame0, frame1,
  			     keypoints0, keypoints1,
  			     features0, features1,
  			     true, &f0_to_f1);
  hrt.stop();
#ifdef VISUALIZE
  eval->vis_.removeAllPointClouds();
  rgbd::Cloud::Ptr 
    cloud0(new rgbd::Cloud), 
    cloud1(new rgbd::Cloud), 
    cloud0_trans(new rgbd::Cloud);
  model.frameToCloud(frame0, cloud0.get());
  model.frameToCloud(frame1, cloud1.get());
  pcl::transformPointCloud(*cloud0, *cloud0_trans, f0_to_f1.cast<float>());
  eval->vis_.addPointCloud(cloud0_trans, "cloud0_trans");
  eval->vis_.addPointCloud(cloud1, "cloud1_trans");
  eval->vis_.spin();
#endif
  cout << "Ground truth transform: " << endl << mat << endl;
  cout << "Estimated transform: " << endl << f0_to_f1.matrix() << endl;
  double translation_error = (f0_to_f1.translation() - mat.block<3, 1>(0, 3)).lpNorm<1>();
  cout << "1-norm of translation difference: " << translation_error << endl;
  double rotation_error = (f0_to_f1.rotation() - mat.block<3, 3>(0, 0)).lpNorm<1>();
  cout << "1-norm of rotation matrix difference: " << rotation_error << endl;
  
  ++eval->num_alignments_;
  if(!found)
    ++eval->num_guessed_failed_;
  eval->total_translation_error_ += translation_error;
  eval->total_rotation_error_ += rotation_error;
  eval->total_seconds_ += hrt.getSeconds();
}

int main(int argc, char** argv)
{
  string alignments_path;
  string model_path;
  
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("alignments", bpo::value<string>(&alignments_path)->required(), "Directory of alignments produced by visualize_frame_alignment.")
    ("model", bpo::value<string>(&model_path)->required(), "PrimeSenseModel")
    ;

  p.add("alignments", 1).add("model", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " ALIGNMENTS MODEL [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Loading " << model_path << endl;
  PrimeSenseModel model;
  model.load(model_path);
  cout << model.status() << endl;
  
  vector<string> paths;
  bfs::directory_iterator it(alignments_path), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    string path = alignments_path + "/" + p.leaf();
    if(path == model_path) continue;
    ROS_ASSERT(bfs::is_directory(path));
    paths.push_back(path);
  }

  sort(paths.begin(), paths.end());
  AlignmentEvaluation eval;
  for(size_t i = 0; i < paths.size(); ++i) {
    cout << paths[i] << endl;
    evaluate(paths[i], model, &eval);
  }
  cout << eval.status() << endl;
  
  return 0;
}
