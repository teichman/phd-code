#include <boost/program_options.hpp>
#include <sentinel/RecordingRequest.h>
#include <jarvis/Detection.h>
#include <name_mapping/name_mapping.h>
#include <online_learning/dataset.h>
#include <ros/ros.h>
 
using namespace std;

class RecordingRequester
{
public:
  RecordingRequester(std::string class_name, double threshold, double seconds = 1);
  
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_; 
  ros::Subscriber sub_;
  std::string class_name_;
  double threshold_;
  double seconds_;

  void detectionCallback(const jarvis::Detection& msg);
};

RecordingRequester::RecordingRequester(std::string class_name, double threshold, double seconds) :
  class_name_(class_name),
  threshold_(threshold),
  seconds_(seconds)
{
  pub_ = nh_.advertise<sentinel::RecordingRequest>("recording_requests", 0);
  sub_ = nh_.subscribe("detections", 0, &RecordingRequester::detectionCallback, this);
}

void RecordingRequester::detectionCallback(const jarvis::Detection& det)
{
  NameMapping cmap(det.cmap);
  Label tpred(det.track_prediction);
  
  if(!cmap.hasName(class_name_))
    return;
  size_t id = cmap.toId(class_name_);
  if(tpred(id) > threshold_) {
    sentinel::RecordingRequest rr;
    rr.timeout = ros::Time::now() + ros::Duration(seconds_);
    rr.tag = class_name_;
    pub_.publish(rr);
    cout << "Published recording request for class " << class_name_ << ".  " << rr.timeout << endl;
    
  }
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string class_name;
  double threshold;
  double seconds;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("class-name,c", bpo::value(&class_name)->required(), "")
    ("threshold,t", bpo::value(&threshold)->required(), "")
    ("seconds,s", bpo::value(&seconds)->default_value(3), "Number of seconds to record after no more detections are seen.")
    ;

  p.add("class-name", 1);
  p.add("threshold", 1);
  p.add("seconds", 1);


  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] CLASS_NAME THRESHOLD SECONDS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ros::init(argc, argv, "request_detection_recording");

  RecordingRequester rr(class_name, threshold, seconds);
  ros::spin();

  return 0;
}


