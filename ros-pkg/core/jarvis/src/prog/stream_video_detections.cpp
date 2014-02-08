#include <jarvis/video_streaming_sentinel.h>
#include <boost/program_options.hpp>

using namespace std;

int main(int argc, char** argv)
{
  // We need NoSigintHandler so that the OpenNI2Interface will
  // get SIGINT and shut down properly.
  ros::init(argc, argv, "sentinel", ros::init_options::NoSigintHandler);


  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  double update_interval;
  double occupancy_threshold;
  int raytracing_threshold;
  double detection_threshold;
  string color_resolution;
  string depth_resolution;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("update-interval,u", bpo::value(&update_interval)->default_value(0.1), "How often to update, in seconds")
    ("occupancy-threshold,o", bpo::value(&occupancy_threshold)->default_value(60), "")
    ("raytracing-threshold,r", bpo::value(&raytracing_threshold)->default_value(10), "")
    ("detection-threshold,d", bpo::value(&detection_threshold)->default_value(0.01), "Foreground must be greater than this percent in [0, 1] of the image to send a Foreground message.")
    ("color-res", bpo::value(&color_resolution), "QVGA or VGA")
    ("depth-res", bpo::value(&depth_resolution), "QVGA or VGA")
    ("visualize", "Show extra visualization")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  OpenNI2Interface::Resolution color_res = OpenNI2Interface::QVGA;
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
  OpenNI2Interface::Resolution depth_res = OpenNI2Interface::QVGA;
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

  VideoStreamingSentinel sen(update_interval,
                             occupancy_threshold, raytracing_threshold,
                             detection_threshold, opts.count("visualize"),
                             color_res, depth_res);
  
  sen.run();

  return 0;
}
