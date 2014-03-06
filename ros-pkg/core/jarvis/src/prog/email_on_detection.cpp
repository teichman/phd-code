#include <ros/package.h>
#include <boost/program_options.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <jarvis/Detection.h>
#include <online_learning/dataset.h>
#include <name_mapping/name_mapping.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class EmailReactor
{
public:
  EmailReactor(std::string address, std::string cname, double min_period);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber det_sub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  cv_bridge::CvImage cv_img_;
  std::string address_;
  std::string cname_;
  double min_period_;
  int count_;
  double det_ts_;  // Most recent detection timestamp, in seconds
  double send_ts_;  // in seconds
  bool send_;

  void detectionCallback(const jarvis::Detection& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void processDetection(double ts);
};

EmailReactor::EmailReactor(std::string address, std::string cname, double min_period) :
  it_(nh_),
  address_(address),
  cname_(cname),
  min_period_(min_period),
  count_(0),
  det_ts_(0),
  send_ts_(0),
  send_(false)
{
  det_sub_ = nh_.subscribe("detections", 0, &EmailReactor::detectionCallback, this);
  img_sub_ = it_.subscribe("image", 0, &EmailReactor::imageCallback, this);
}

void EmailReactor::detectionCallback(const jarvis::Detection& msg)
{
  NameMapping cmap(msg.cmap);
  if(!cmap.hasName(cname_)) {
    cout << "[EmailReactor]  Class to trigger on does not exist in Detection cmap." << endl;
    return;
  }

  // Mysteriously, this causes the program to duplicate itself, at least according to ps aux.
  // Unsurprisingly, when this happens, nothing works.
  // I am not making this up.  WTGDF.
  // Label tpred(msg.track_prediction);
  // cout << "Label: " << tpred.transpose() << endl;

  // Fortunately, we can use the vector<float> directly instead as a workaround for now.
  if(msg.track_prediction[cmap.toId(cname_)] > 0) {
    cout << "Track prediction: ";
    copy(msg.track_prediction.begin(), msg.track_prediction.end(), ostream_iterator<float>(cout, " "));
    cout << endl;

    processDetection(msg.header.stamp.toSec());
  }
}

void EmailReactor::processDetection(double ts)
{
  // Reset the count if we haven't seen anything in a long time.
  if(ts - det_ts_ > 10)
    count_ = 0;
  det_ts_ = ts;
  
  // If we've haven't yet seen enough of the object in question, don't do anything.
  ++count_;
  if(count_ < 30)
    return;

  // If we've sent an email too recently, don't do anything.
  if(ts - send_ts_ < min_period_)
    return;

  // Otherwise, send an email.
  send_ts_ = ts;
  send_ = true;
}

void EmailReactor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
  // cv::imshow("Image", cv_ptr->image);
  // cv::waitKey(3);
  // cout << "New image: " << setprecision(16) << msg->header.stamp.toSec() << endl;
  // processDetection(msg->header.stamp.toSec());
  
  if(send_) {
    cout << "Emailing... " << endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
    string filename = ".emailreactor.tmp.png";
    cv::imwrite(filename, cv_ptr->image);
    ostringstream oss;
    oss << "python " << ros::package::getPath("jarvis") << "/src/python/send_photo.py "
        << filename << " anon.aoeu@gmail.com " << address_ << " Detection of " << cname_
        << " --body This is the body."; 
    string cmd = oss.str();
    cout << "Running command: " << endl << cmd << endl;
    int rv = -1;
    while(rv != 0) {
      rv = system(cmd.c_str());
      if(rv != 0)
        ROS_WARN("Failed to send email.");
      usleep(5e-6);
    }
    
    send_ = false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "email_on_detection");
  
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string address;
  double min_period;
  string cname;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("address", bpo::value(&address), "Who to send email to")
    ("min-period,t", bpo::value(&min_period)->default_value(30), "This program will email you no more than once every X seconds")
    ("class", bpo::value(&cname)->required(), "What object to respond to")
    ;

  p.add("address", 1);
  p.add("class", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] EMAIL_ADDRESS CLASS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Starting up EmailReactor" << endl;
  EmailReactor er(address, cname, min_period);
  ros::spin();
  
  return 0;
}
