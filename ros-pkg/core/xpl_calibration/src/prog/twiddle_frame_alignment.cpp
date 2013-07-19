#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pipeline/params.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/package.h>
#include <iomanip>
#include <eigen_extensions/random.h>
#include <xpl_calibration/frame_aligner.h>
#include <pipeline/twiddler.h>

using namespace std;
using namespace pl;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

class FrameAlignmentTwiddler : public Twiddler
{
public:
  Results evaluate(const Params& params, string evalpath);
  Params generateParamVariation(Params params) const;
  double objective(const Results& results) const;
};

double FrameAlignmentTwiddler::objective(const Results& results) const
{
  // cm + degrees.
  double objective = 0;
  objective += results["rotation_error"] * 180.0 / M_PI;
  objective += results["translation_error"] * 100;

  // Timing constraint.  Should be tightened up after optimization.
  if(results["time"] > 30)
    objective = numeric_limits<double>::max();
  
  return objective;
}

Twiddler::Results FrameAlignmentTwiddler::evaluate(const Params& params, std::string evalpath)
{
  ostringstream oss;
  oss << ros::package::getPath("xpl_calibration") << "/bin/evaluate_frame_alignment alignments alignments/default_primesense_model --params " << evalpath << "/params.txt > " << evalpath << "/output.txt";
  int retval;
  retval = system(oss.str().c_str()); ROS_ASSERT(retval == 0);
  retval = system(("tail -n4 " + evalpath + "/output.txt | awk '{print $NF}' > " + evalpath + "/numbers.txt").c_str()); ROS_ASSERT(retval == 0);

  ifstream file((evalpath + "/numbers.txt").c_str());
  ROS_ASSERT(file.is_open());
  Twiddler::Results results;
  double translation_error;
  file >> translation_error;
  results["translation_error"] = translation_error;
  
  file >> results["rotation_error"];
  file >> results["time"];
  file.close();
  return results;
}

Params FrameAlignmentTwiddler::generateParamVariation(Params params) const
{
  vector<string> moves;
  vector<double> weights;
  moves.push_back("color_weight");
  weights.push_back(1);
  moves.push_back("cn_weight");
  weights.push_back(1);
  moves.push_back("edge_weight");
  weights.push_back(1);
  moves.push_back("hue_weight");
  weights.push_back(1);
  moves.push_back("rgb_weight");
  weights.push_back(1);
  moves.push_back("all_color_terms");
  weights.push_back(1);
  moves.push_back("canny");
  weights.push_back(1);

  Eigen::VectorXd eigweights(weights.size());
  for(size_t i = 0; i < weights.size(); ++i)
    eigweights(i) = weights[i];
  int idx = eigen_extensions::weightedSample(eigweights);
  string move = moves[idx];

  if(move == "color_weight" || move == "all_color_terms") {
    vector<double> color_weight_values;
    color_weight_values.push_back(0);
    color_weight_values.push_back(0.01);
    color_weight_values.push_back(0.02);
    color_weight_values.push_back(0.05);
    color_weight_values.push_back(0.1);
    params.set<double>("color_weight", color_weight_values[rand() % color_weight_values.size()]);
  }
  if(move == "cn_weight" || move == "all_color_terms") {
    vector<double> cn_weight_values;
    cn_weight_values.push_back(0);
    cn_weight_values.push_back(0.01);
    cn_weight_values.push_back(0.02);
    cn_weight_values.push_back(0.05);
    cn_weight_values.push_back(0.1);
    params.set<double>("cn_weight", cn_weight_values[rand() % cn_weight_values.size()]);
  }
  if(move == "hue_weight" || move == "all_color_terms") {
    vector<double> hue_weight_values;
    hue_weight_values.push_back(0);
    hue_weight_values.push_back(0.01);
    hue_weight_values.push_back(0.02);
    hue_weight_values.push_back(0.05);
    hue_weight_values.push_back(0.1);
    params.set<double>("hue_weight", hue_weight_values[rand() % hue_weight_values.size()]);
  }
  if(move == "rgb_weight" || move == "all_color_terms") {
    vector<double> rgb_weight_values;
    rgb_weight_values.push_back(0);
    rgb_weight_values.push_back(0.01);
    rgb_weight_values.push_back(0.02);
    rgb_weight_values.push_back(0.05);
    rgb_weight_values.push_back(0.1);
    params.set<double>("rgb_weight", rgb_weight_values[rand() % rgb_weight_values.size()]);
  }
  if(move == "edge_weight" || move == "all_color_terms") {
    vector<double> edge_weight_values;
    edge_weight_values.push_back(0);
    edge_weight_values.push_back(0.01);
    edge_weight_values.push_back(0.02);
    edge_weight_values.push_back(0.05);
    edge_weight_values.push_back(0.1);
    params.set<double>("edge_weight", edge_weight_values[rand() % edge_weight_values.size()]);
  }

  // Don't bother touching these if we aren't using the edges anyway.
  // Twiddler will see that params haven't changed and won't evaluate them if this is the case.
  if(move == "canny" && params.get<double>("edge_weight") != 0) {
    vector<int> radius_values;
    radius_values.push_back(1);
    // 5 caused an opencv crash.  What are acceptable params for this?  Disabling for now.
    // radius_values.push_back(2);
    // radius_values.push_back(5);
    params.set<int>("canny_kernel_radius", radius_values[rand() % radius_values.size()]);
    vector<int> lower_thresh_values;
    lower_thresh_values.push_back(75);
    lower_thresh_values.push_back(50);
    lower_thresh_values.push_back(100);
    params.set<int>("canny_lower_thresh", lower_thresh_values[rand() % lower_thresh_values.size()]);
    vector<int> upper_thresh_values;
    upper_thresh_values.push_back(params.get<int>("canny_lower_thresh") + 25);
    upper_thresh_values.push_back(params.get<int>("canny_lower_thresh") + 50);
    upper_thresh_values.push_back(params.get<int>("canny_lower_thresh") + 75);
    params.set<int>("canny_upper_thresh", upper_thresh_values[rand() % upper_thresh_values.size()]);
  }
  
  return params;
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("output,o", bpo::value<string>()->required(), "Directory to be created in which output will be stored.")
    ;

  p.add("output", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << "OUTPUT [OPTS]" << endl;
    cout << "       You must be in a directory that contains alignments/." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  srand(time(0));
  FrameAlignmentTwiddler twiddler;
  string path = opts["output"].as<string>();
  if(bfs::exists(path)) {
    cout << "Resuming Twiddler run in " << path << endl;
    twiddler.load(path);
  }
  else {
    cout << "Initializing new Twiddler in " << path << endl;
    twiddler.initialize(FrameAligner::defaultParams(), path);
  }
  twiddler.twiddle();
  
  return 0;
}

