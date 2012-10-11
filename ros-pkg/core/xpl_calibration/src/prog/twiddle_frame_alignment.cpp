#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pipeline/params.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/package.h>
#include <iomanip>
#include <xpl_calibration/frame_aligner.h>
#include <pipeline/twiddler.h>

using namespace std;
using namespace pipeline;
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
  oss << ros::package::getPath("xpl_calibration") << "/bin/evaluate_frame_alignment one_alignment one_alignment/default_primesense_model --params " << evalpath << "/params.txt > " << evalpath << "/output.txt";
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
  vector<double> color_weight_values;
  color_weight_values.push_back(0);
  color_weight_values.push_back(1e-3);
  color_weight_values.push_back(1e-2);
  color_weight_values.push_back(1e-1);
  color_weight_values.push_back(1);
  
  params.set<double>("color_weight", color_weight_values[rand() % color_weight_values.size()]);
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

  FrameAlignmentTwiddler twiddler;
  twiddler.run(FrameAligner::defaultParams(), opts["output"].as<string>());
  
  return 0;
}

