#include <jarvis/cannon_reactor.h>
#include <eigen_extensions/eigen_extensions.h>
#include <online_learning/dataset.h>
#include <X11/Xlib.h>  // This header causes all kinds of shit if you put it at the top.

using namespace std;
using namespace Eigen;

DiscreteBayesFilter::DiscreteBayesFilter(float cap, double weight, Label prior) :
  cap_(cap),
  weight_(weight),
  prior_(prior)
{
}

void DiscreteBayesFilter::addObservation(jarvis::DetectionConstPtr msg)
{
  frame_predictions_.push_back(Label(msg->frame_prediction));
  
  if(frame_predictions_.size() == 1)
    cumulative_ = msg->frame_prediction;
  else
    cumulative_ += Label(msg->frame_prediction) * (eigen_extensions::vecToEig(msg->centroid) - prev_centroid_).norm() * weight_;

  for(int i = 0; i < cumulative_.rows(); ++i)
    cumulative_[i] = max(-cap_, min(cap_, cumulative_[i]));

  prev_centroid_ = eigen_extensions::vecToEig(msg->centroid);
  prev_sensor_timestamp_ = msg->sensor_timestamp;

  if(prior_.rows() == 0)
    prior_ = VectorXf::Zero(cumulative_.rows());
}

Label DiscreteBayesFilter::trackPrediction() const
{
  // TODO: Label needs to be fixed so that this can be one line.
  Label track_prediction = cumulative_;
  track_prediction += prior_;
  return track_prediction;
}

CannonReactor::CannonReactor(double threshold) :
  threshold_(threshold)
{
  cannon_driver_.detach();  // Run this in its own thread.    
  hrt_.start();
}

CannonReactor::~CannonReactor()
{
  int ret = system("killall pyrocket");
  ROS_ASSERT(ret == 0);
}

void CannonReactor::detectionCallback(jarvis::DetectionConstPtr msg)
{
  NameMapping cmap(msg->cmap);
  if(!cmap.hasName("cat")) {
    ROS_WARN_ONCE("CannonReactor expects detections messages that make predictions about cats.");
    return;
  }

  filters_[msg->track_id].addObservation(msg);

  // -- If the updated track warrants shooting, do so.
  Label pred = filters_[msg->track_id].trackPrediction();
  cout << "[CannonReactor]  Track " << msg->track_id << ": " << pred.transpose() << std::flush;
  if(pred(cmap.toId("cat")) > threshold_) {
    cout << "  *** " << std::flush;
    if(!cannon_driver_.firing()) {
      cout << " FIRING" << std::flush;
      cannon_driver_.fire();
      hrt_.reset();
      hrt_.start();
    }
  }
  cout << endl;

  // -- Prune out any old DiscreteBayesFilters.
  auto it = filters_.begin();
  while(it != filters_.end()) {
    const DiscreteBayesFilter& dbf = it->second;
    if(msg->sensor_timestamp > dbf.timestamp() + 10.0)
      filters_.erase(it++);
    else
      it++;
  }
}

CannonDriver::CannonDriver() :
  firing_(false),
  num_darts_(4)
{
  // -- Start up pyrocket.
  int ret = system("killall -9 pyrocket");
  usleep(1e6);
  ret = system("~/pyrocket/src/pyrocket &");
  ROS_ASSERT(ret == 0);

  // -- Place the mouse cursor over the fire button.
  Display* display = XOpenDisplay(NULL);
  assert(display);
  Window root = DefaultRootWindow(display);
  int row = 150;
  //int col = 10;
  int col = 650;  // jarvis display window + 10
  XWarpPointer(display, 0, root, 0, 0, 0, 0, col, row);
  XCloseDisplay(display);

  // -- Put the cannon in an initial known position at far left and pointed down.
  usleep(1e6);
  ret = system("xdotool keydown Alt; xdotool keydown Down; sleep 3; xdotool keyup Down; xdotool keyup Alt");
  ROS_ASSERT(ret == 0);
  ret = system("xdotool keydown Alt; xdotool keydown Left; sleep 5; xdotool keyup Left; xdotool keyup Alt");
  ROS_ASSERT(ret == 0);
}

void CannonDriver::_run()
{
  while(true) {
    usleep(1e5);
    if(firing_) {
      sendFireMessage();
      firing_ = false;
    }
  }
}

void CannonDriver::sendFireMessage()
{
  if(num_darts_ == 0) {
    cout << "[CannonDriver]  Out of ammo..." << endl;
    return;
  }
  --num_darts_;
  
  int ret;

  // -- Rotate the cannon into firing position.
  ret = system("xdotool keydown Alt; xdotool keydown Right; sleep 2.5; xdotool keyup Right; xdotool keyup Alt");
  ROS_ASSERT(ret == 0);
  ret = system("xdotool keydown Alt; xdotool keydown Up; sleep 1; xdotool keyup Up; xdotool keyup Alt");
  ROS_ASSERT(ret == 0);

  // -- Fire.
  ret = system("sleep 3; xdotool mousedown 1; sleep 0.2; xdotool mouseup 1; sleep 5");
  ROS_ASSERT(ret == 0);

  // -- Return to original position at far left.
  ret = system("xdotool keydown Alt; xdotool keydown Down; sleep 2; xdotool keyup Down; xdotool keyup Alt");
  ROS_ASSERT(ret == 0);
  ret = system("xdotool keydown Alt; xdotool keydown Left; sleep 3; xdotool keyup Left; xdotool keyup Alt");
  ROS_ASSERT(ret == 0);
}
