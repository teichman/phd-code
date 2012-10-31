#include <boost/program_options.hpp>
#include <rgbd_sequence/openni_stream_recorder.h>

using namespace std;
using namespace rgbd;

int main(int argc, char** argv)
{
  // -- Parse args.
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "")
    ("device", bpo::value<string>()->required(), "Device type.  \"xpl\" or \"kinect\"")
    ("id", bpo::value<int>()->required(), "Device id")
    ("register", "Register depth to rgb data")
    ("fake-rgb", "Don't actually record rgb data")
    ("resolution", bpo::value<string>()->default_value("VGA"), "VGA (640x480) or QVGA (320x240)")
    ("output,o", bpo::value<string>(), "Directory to put new sequences in")
    ;

  bpo::positional_options_description p;
  p.add("device", 1);
  p.add("id", 2);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << endl;
    cout << "Usage: record_stream_openni DEVICE ID [OPTS]" << endl;
    cout << opts_desc << endl << endl;
    return 1;
  }
  bpo::notify(opts);

  ROS_ASSERT(!(opts.count("register") && opts.count("fake-rgb")));
  
  OpenNIStreamRecorder rec(opts["device"].as<string>(),
			   opts["id"].as<int>(),
			   opts["resolution"].as<string>(),
			   opts.count("fake-rgb"),
			   opts.count("register"));

  if(opts.count("output"))
    rec.sequences_dir_ = opts["output"].as<string>();
  cout << "Saving new sequences in " << rec.sequences_dir_ << endl;
			   
  rec.run();

  return 0;
}
