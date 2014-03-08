#include <sentinel/sentinel.h>
#include <boost/program_options.hpp>

#if JARVIS_PROFILE
#include <gperftools/profiler.h>
#endif

using namespace std;

int main(int argc, char** argv)
{
  // We need NoSigintHandler so that the OpenNI2Interface will
  // get SIGINT and shut down properly.
  ros::init(argc, argv, "sentinel", ros::init_options::NoSigintHandler);


  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string sensor_id;
  double update_interval;
  double occupancy_threshold;
  int raytracing_threshold;
  double detection_threshold;
  size_t max_training_imgs;
  string color_resolution;
  string depth_resolution;
  string recording_dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sensor-id", bpo::value(&sensor_id), "e.g. xpl07")
    ("update-interval,u", bpo::value(&update_interval)->default_value(0.1), "How often to update, in seconds")
    ("occupancy-threshold,o", bpo::value(&occupancy_threshold)->default_value(600), "")
    ("raytracing-threshold,r", bpo::value(&raytracing_threshold)->default_value(10), "")
    ("detection-threshold,d", bpo::value(&detection_threshold)->default_value(0), "Foreground must be greater than this percent of the image to send a Foreground message.")
    ("color-res", bpo::value(&color_resolution), "QVGA or VGA")
    ("depth-res", bpo::value(&depth_resolution), "QVGA or VGA")
    ("visualize", "Show extra visualization")
    ("record-all-motion", "")
    ("recording-dir", bpo::value(&recording_dir)->required(), "Directory to save recordings to")
    ;

  p.add("sensor-id", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] SENSOR_ID" << endl;
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

  ROSStreamingSentinel sen(sensor_id, recording_dir, update_interval,
                           occupancy_threshold, raytracing_threshold,
                           detection_threshold, opts.count("visualize"),
                           opts.count("record-all-motion"),
                           color_res, depth_res);

  #if JARVIS_PROFILE
  ProfilerStart("sentinel.prof");
  #endif
  
  sen.run();

  #if JARVIS_PROFILE
  ProfilerStop();
  #endif

  return 0;
}
