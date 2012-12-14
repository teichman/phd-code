#include <boost/program_options.hpp>
#include <xpl_calibration/discrete_depth_distortion_model.h>

using namespace std;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string path;
  string dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("intrinsics", bpo::value<string>(&path)->required(), "DiscreteDepthDistortionModel to examine.")
    ("output", bpo::value<string>(&dir)->default_value("."), "Output directory.  Must exist.")
    ;

  p.add("intrinsics", 1).add("output", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " INTRINSICS [ OUTPUT_PATH ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  DiscreteDepthDistortionModel dddm;
  cout << "Loading DiscreteDepthDistortionModel at " << path << endl;
  dddm.load(path);
  dddm.visualize(dir);
  cout << "Saved visualization to " << dir << endl;
  
  return 0;
}
