#include <sentinel/RecordingRequest.h>
#include <boost/program_options.hpp>
#include <ros/ros.h>

using namespace std;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string tag;
  double secs;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("tag", bpo::value(&tag)->required(), "")
    ("secs", bpo::value(&secs)->required(), "")
    ;

  p.add("tag", 1);
  p.add("secs", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] TAG SECS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ros::init(argc, argv, "request_recording");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sentinel::RecordingRequest>("recording_requests", 0);
  sentinel::RecordingRequest msg;
  msg.timeout = ros::Time::now() + ros::Duration(secs);
  msg.tag = tag;
  usleep(1e5);  // Without a delay, the receiver sometimes doesn't get anything.  Weird.
  pub.publish(msg);
  cout << "Published." << endl;

  return 0;
}
