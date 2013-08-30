#include <sentinel/sentinel.h>
#include <boost/program_options.hpp>

using namespace std;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;
  
  string name;
  string color_resolution;
  string depth_resolution;
  double threshold;
  double save_interval;
  double update_interval;
  size_t max_training_imgs;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("name", bpo::value(&name)->required(), "Data will be saved to .sentinel-name")
    ("visualize", "Whether to show the rgb stream")
    ("color-res", bpo::value(&color_resolution), "")
    ("depth-res", bpo::value(&depth_resolution), "")
    ("threshold,t", bpo::value(&threshold)->default_value(0.03), "")
    ("save-interval,s", bpo::value(&save_interval)->default_value(1), "How often to save, in seconds")
    ("update-interval,u", bpo::value(&update_interval)->default_value(1), "How often to update, in seconds")
    ("max-training-imgs,m", bpo::value(&max_training_imgs)->default_value(1000), "")
    ;

  p.add("name", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] NAME" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }


  OpenNI2Interface::Resolution color_res = OpenNI2Interface::VGA;
  if(opts.count("color-res")) {
    if(color_resolution == "QVGA" || color_resolution == "qvga")
      color_res = OpenNI2Interface::QVGA;
    else if(color_resolution == "VGA" || color_resolution == "vga")
      color_res = OpenNI2Interface::VGA;
    else {
      cout << "Unrecognized resolution \"" << color_res << "\"." << endl;
      return 1;
    }
  }
  OpenNI2Interface::Resolution depth_res = OpenNI2Interface::VGA;
  if(opts.count("depth-res")) {
    if(depth_resolution == "QVGA" || depth_resolution == "qvga")
      depth_res = OpenNI2Interface::QVGA;
    else if(depth_resolution == "VGA" || depth_resolution == "vga")
      depth_res = OpenNI2Interface::VGA;
    else {
      cout << "Unrecognized resolution \"" << depth_res << "\"." << endl;
      return 1;
    }
  }
  
  DiskStreamingSentinel sen("sentinel-" + name, save_interval,
                            update_interval, max_training_imgs,
                            threshold, opts.count("visualize"),
                            color_res, depth_res);
  sen.run();

  return 0;
}
