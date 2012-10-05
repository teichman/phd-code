#include <boost/program_options.hpp>
#include <xpl_calibration/primesense_slam.h>

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
  std::string individual_results_;

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
    oss << "Mean translation error (meters): " << total_translation_error_ / num_alignments_ << endl;
    oss << "Mean rotation difference (radians): " << total_rotation_error_ / num_alignments_ << endl;
    oss << "Mean alignment time: " << total_seconds_ / num_alignments_ << " seconds." << endl;
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

  ostringstream oss;
  oss << eval->individual_results_ << endl;
  
  oss << "===  " << path << "  ===" << endl;
  oss << "Guessed that alignment was found: " << found << endl;
  // oss << "Ground truth transform: " << endl << mat << endl;
  // oss << "Estimated transform: " << endl << f0_to_f1.matrix() << endl;
  double translation_error = (f0_to_f1.translation() - mat.block<3, 1>(0, 3)).norm();
  oss << "L2 norm of translation difference: " << translation_error << endl;

  double rx, ry, rz, gtrx, gtry, gtrz;
  double tx, ty, tz;  // ignored.
  generateXYZYPR(f0_to_f1.cast<float>(), rx, ry, rz, tx, ty, tz);
  generateXYZYPR(Affine3f(mat.cast<float>()), gtrx, gtry, gtrz, tx, ty, tz);
  double rotation_error = fabs(rx - gtrx) + fabs(ry - gtry) + fabs(rz - gtrz);
  oss << "Total absolute rotation difference (radians): " << rotation_error << endl;
  eval->individual_results_ = oss.str();
  
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
    if(bfs::is_directory(path))
      paths.push_back(path);
  }

  sort(paths.begin(), paths.end());
  AlignmentEvaluation eval;
  for(size_t i = 0; i < paths.size(); ++i) {
    cout << paths[i] << endl;
    evaluate(paths[i], model, &eval);
  }

  cout << endl << endl << endl;
  cout << "============================== Individual results " << endl;
  cout << eval.individual_results_ << endl;
  cout << "============================== Overall results " << endl;
  cout << eval.status() << endl;
  
  return 0;
}
