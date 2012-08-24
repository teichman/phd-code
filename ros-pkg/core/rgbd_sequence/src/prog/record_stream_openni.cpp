#include <boost/program_options.hpp>
#include <rgbd_sequence/openni_stream_recorder.h>

using namespace std;
using namespace rgbd;
namespace bpo = boost::program_options;  

int main(int argc, char** argv)
{
  // -- Parse args.
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("device", bpo::value<string>()->required(), "Device type.  \"xpl\" or \"kinect\"")
    ("id", bpo::value<int>()->required(), "Device id")
    ("register", "Register depth to rgb data")
    ("fake-rgb", "Don't actually record rgb data")
    ;

  bpo::positional_options_description p;
  p.add("device", 1);
  p.add("id", 2);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  if(opts.count("help")) {
    cout << "Usage: record_stream_openni DEVICE ID [OPTS]" << endl;
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  ROS_ASSERT(!(opts.count("register") && opts.count("fake-rgb")));
  
  OpenNIStreamRecorder rec(opts["device"].as<string>(),
			   opts["id"].as<int>(),
			   "VGA",
			   opts.count("fake-rgb"),
			   opts.count("register"));
			   
  rec.run();

  return 0;
}
