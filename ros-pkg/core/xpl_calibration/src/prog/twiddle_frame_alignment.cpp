#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pipeline/params.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/package.h>
#include <iomanip>
#include <xpl_calibration/frame_aligner.h>

using namespace std;
using namespace pipeline;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  //("script", bpo::value<string>()->required(), "Runs the evaluation.")
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("output,o", bpo::value<string>()->required(), "Directory to be created in which output will be stored.")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    //cout << "Usage: " << bfs::basename(argv[0]) << " SCRIPT BASE_PARAMS [OPTS] -o OUTPUT" << endl;
    cout << "Usage: " << bfs::basename(argv[0]) << "[OPTS] -o OUTPUT" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Set up all param variations.
  Params base_params = FrameAligner::defaultParams();
  vector<Params> allparams;

  // vector<double> color_weight_values;
  // color_weight_values.push_back(0.001);
  // color_weight_values.push_back(0.01);
  // color_weight_values.push_back(0.1);
  // for(size_t i = 0; i < color_weight_values.size(); ++i) {
  //   Params p = base_params;
  //   p.set("color_weight", color_weight_values[i]);
  //   allparams.push_back(p);
  // }

  vector<double> rgb_weight_values;
  rgb_weight_values.push_back(0.1);
  rgb_weight_values.push_back(0.2);
  rgb_weight_values.push_back(0.5);
  rgb_weight_values.push_back(1.0);
  for(size_t i = 0; i < rgb_weight_values.size(); ++i) {
    Params p = base_params;
    p.set("rgb_weight", rgb_weight_values[i]);
    allparams.push_back(p);
  }

  vector<double> cn_weight_values;
  cn_weight_values.push_back(0.1);
  cn_weight_values.push_back(0.2);
  cn_weight_values.push_back(0.5);
  cn_weight_values.push_back(1.0);
  for(size_t i = 0; i < cn_weight_values.size(); ++i) {
    Params p = base_params;
    p.set("cn_weight", cn_weight_values[i]);
    allparams.push_back(p);
  }

  vector<double> hue_weight_values;
  hue_weight_values.push_back(0.1);
  hue_weight_values.push_back(0.2);
  hue_weight_values.push_back(0.5);
  hue_weight_values.push_back(1.0);
  for(size_t i = 0; i < hue_weight_values.size(); ++i) {
    Params p = base_params;
    p.set("hue_weight", hue_weight_values[i]);
    allparams.push_back(p);
  }

  vector<double> edge_weight_values;
  edge_weight_values.push_back(0.1);
  edge_weight_values.push_back(0.2);
  edge_weight_values.push_back(0.5);
  edge_weight_values.push_back(1.0);
  for(size_t i = 0; i < edge_weight_values.size(); ++i) {
    Params p = base_params;
    p.set("edge_weight", edge_weight_values[i]);
    allparams.push_back(p);
  }
  
  // vector<double> fraction_values;
  // fraction_values.push_back(0.1);
  // fraction_values.push_back(0.25);
  // fraction_values.push_back(0.5);
  // for(size_t i = 0; i < fraction_values.size(); ++i) {
  //   Params p = base_params;
  //   p.set("fraction", fraction_values[i]);
  //   allparams.push_back(p);
  // }

  // vector<double> ransac_max_inlier_dist_values;
  // ransac_max_inlier_dist_values.push_back(0.01);
  // ransac_max_inlier_dist_values.push_back(0.033);
  // ransac_max_inlier_dist_values.push_back(0.05);
  // for(size_t i = 0; i < ransac_max_inlier_dist_values.size(); ++i) {
  //   Params p = base_params;
  //   p.set("ransac_max_inlier_dist", ransac_max_inlier_dist_values[i]);
  //   allparams.push_back(p);
  // }
  
  // -- Set up output directory.
  ROS_ASSERT(!bfs::exists(opts["output"].as<string>()));
  bfs::create_directory(opts["output"].as<string>());

  // -- Run evaluation.
  for(size_t i = 0; i < allparams.size(); ++i) {
    ostringstream oss;
    oss << opts["output"].as<string>() << "/" << setw(4) << setfill('0') << i;
    string path = oss.str();
    bfs::create_directory(path);
    allparams[i].save(path + "/params.txt");

    oss.str("");
    oss << ros::package::getPath("xpl_calibration") << "/bin/evaluate_frame_alignment alignments alignments/default_primesense_model --params " << path << "/params.txt > " << path << "/output.txt";
    int retval = system(oss.str().c_str()); --retval;
  }
  
  return 0;
}

