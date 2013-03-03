#include <xpl_calibration/discrete_depth_distortion_model.h>

using namespace std;
using namespace rgbd;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("distortion-model", bpo::value<string>())
    ;

  bpo::positional_options_description p;
  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: projection_demo [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  DiscreteDepthDistortionModel dddm;
  if(opts.count("distortion-model")) {
    cout << "Using distortion model at " << opts["distortion-model"].as<string>() << endl; 
    dddm.load(opts["distortion-model"].as<string>());
  }

  return 0;
}
