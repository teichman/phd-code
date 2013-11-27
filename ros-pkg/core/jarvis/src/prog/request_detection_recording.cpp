#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <sentinel/RecordingRequest.h>
#include <jarvis/Detection.h>
#include <name_mapping/name_mapping.h>
#include <online_learning/dataset.h>
#include <ros/ros.h>
 
using namespace std;
namespace bfs = boost::filesystem;

class RecordingRequester
{
public:
  RecordingRequester(std::string class_name, double threshold,
                     std::string metadata_path, double seconds = 1);
  ~RecordingRequester();
  
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_; 
  ros::Subscriber sub_;
  std::string class_name_;
  double threshold_;
  std::string metadata_path_;
  double seconds_;
  ofstream output_;

  void detectionCallback(const jarvis::Detection& msg);
};

RecordingRequester::RecordingRequester(std::string class_name, double threshold,
                                       std::string metadata_path, double seconds) :
  class_name_(class_name),
  threshold_(threshold),
  metadata_path_(metadata_path),
  seconds_(seconds)
{
  pub_ = nh_.advertise<sentinel::RecordingRequest>("recording_requests", 0);
  sub_ = nh_.subscribe("detections", 0, &RecordingRequester::detectionCallback, this);

  ROS_ASSERT(!bfs::exists(metadata_path_));
  output_.open(metadata_path_.c_str());
}

RecordingRequester::~RecordingRequester()
{
  output_.close();
}

void RecordingRequester::detectionCallback(const jarvis::Detection& det)
{
  NameMapping cmap(det.cmap);
  Label tpred(det.track_prediction);
  
  if(!cmap.hasName(class_name_))
    return;
  
  size_t id = cmap.toId(class_name_);
  if(tpred(id) > 0) {
    cout << "[RecordingRequester] " << det.header.stamp << " " << class_name_ << " " << tpred(id) << ".";
    if(tpred(id) > threshold_) {
      sentinel::RecordingRequest rr;
      rr.timeout = det.header.stamp + ros::Duration(seconds_);
      rr.tag = class_name_;
      pub_.publish(rr);
      cout << "   Published.  " << rr.timeout;
    }
    cout << endl;
  }

  // -- Record metadata to a text file.  We'll use this to generate a video with bounding boxes.
  //    (All classifications are sent within the Detection messages, not just positive detections.)
  output_ << fixed << setprecision(16) << setw(16) << setfill('0') << det.header.stamp.toSec() << " "
          << det.track_id << " " 
          << det.upper_left.x << " " << det.upper_left.y << " "
          << det.lower_right.x << " " << det.lower_right.y;
  for(size_t i = 0; i < det.frame_prediction.size(); ++i)
    output_ << " " << det.cmap[i] << " " << det.frame_prediction[i];
  output_ << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "request_detection_recording");
  
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string class_name;
  double threshold;
  string metadata_dir;
  double seconds;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("class-name,c", bpo::value(&class_name)->required(), "")
    ("threshold,t", bpo::value(&threshold)->required(), "")
    ("metadata-dir,d", bpo::value(&metadata_dir)->required(), "Where to store the detections file.  Should be a directory that exists.  The file will be added to it.")
    ("seconds,s", bpo::value(&seconds)->default_value(3), "Number of seconds to record after no more detections are seen.")
    ;

  p.add("class-name", 1);
  p.add("threshold", 1);
  p.add("metadata-dir", 1);
  p.add("seconds", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] CLASS_NAME THRESHOLD METADATA_DIR SECONDS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ostringstream oss;
  oss << metadata_dir << "/detections_metadata-" << fixed << setprecision(16) << setw(16) << setfill('0') << ros::Time::now().toSec() << ".txt";
  RecordingRequester rr(class_name, threshold, oss.str(), seconds);
  ros::spin();

  return 0;
}


