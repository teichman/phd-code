#include <matplotlib_interface/matplotlib_interface.h>
#include <jarvis/cannon_reactor.h>
#include <eigen_extensions/eigen_extensions.h>
#include <online_learning/dataset.h>
#include <sentinel/RecordingRequest.h>
#include <X11/Xlib.h>  // This header causes all kinds of shit if you put it at the top.

using namespace std;
using namespace Eigen;

CannonReactor::CannonReactor(size_t min_num_frames, double threshold) :
  min_num_frames_(min_num_frames),
  threshold_(threshold)
{
  pub_ = nh_.advertise<sentinel::RecordingRequest>("recording_requests", 0);
  cannon_driver_.detach();  // Run this in its own thread.    
  hrt_.start();
}

CannonReactor::~CannonReactor()
{
}

void CannonReactor::detectionCallback(jarvis::DetectionConstPtr msg)
{
  // -- Only consider detections that have been seen for at least some number of frames.
  if(msg->num_frames < min_num_frames_)
    return;

  // -- We're only shooting cats for now.
  NameMapping cmap(msg->cmap);
  if(!cmap.hasName("cat")) {
    ROS_WARN_ONCE("CannonReactor expects detections messages that make predictions about cats.");
    return;
  }
  
  // -- If we're sure it's a cat, fire the cannon.
  Label tpred(msg->track_prediction);
  if(tpred(cmap.toId("cat")) > 0) { 
    cout << "[CannonReactor]  Track " << msg->track_id << ", " << msg->num_frames << " frames.  Track prediction: " << tpred.transpose() << std::flush;
    if(tpred(cmap.toId("cat")) > threshold_) {
      cout << "  *** " << std::flush;
      if(cannon_driver_.ammo() == 0)
        cout << " Out of ammo." << std::flush;
      else if(!cannon_driver_.firing()) {
        cout << " FIRE!" << std::flush;
        cannon_driver_.fire();
        hrt_.reset();
        hrt_.start();
      }
    }
    cout << endl;
  }
  
  // -- If we're firing, send a message telling Sentinel to record.
  //    Record the entire time we're firing, plus 3 seconds afterwards.
  if(cannon_driver_.firing()) {
    sentinel::RecordingRequest msg;
    msg.timeout = ros::Time::now() + ros::Duration(3);
    msg.tag = "firing";
    pub_.publish(msg);
  }
}

GUICannonDriver::GUICannonDriver() :
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
  // usleep(1e6);
  // ret = system("xdotool keydown Alt; xdotool keydown Down; sleep 3; xdotool keyup Down; xdotool keyup Alt");
  // ROS_ASSERT(ret == 0);
  // ret = system("xdotool keydown Alt; xdotool keydown Left; sleep 5; xdotool keyup Left; xdotool keyup Alt");
  // ROS_ASSERT(ret == 0);
}

void GUICannonDriver::_run()
{
  while(true) {
    usleep(1e5);
    if(firing_) {
      sendFireMessage();
      firing_ = false;
    }
  }
}

void sys(std::string str)
{
  int ret = system(str.c_str());
  ROS_ASSERT(ret == 0);
  usleep(5e4);
}

void GUICannonDriver::sendFireMessage()
{
  if(num_darts_ == 0) {
    cout << "[GUICannonDriver]  Out of ammo..." << endl;
    return;
  }
  --num_darts_;
  
  // -- Rotate the cannon into firing position.
  sys("xdotool keydown Alt");
  sys("xdotool keydown Right");
  usleep(2.5e6);
  sys("xdotool keyup Right");
  sys("xdotool keyup Alt");
  
  sys("xdotool keydown Alt");
  sys("xdotool keydown Up");
  usleep(1e6);
  sys("xdotool keyup Up");
  sys("xdotool keyup Alt");

  // -- Fire.
  usleep(1e6);
  sys("xdotool mousedown 1");
  usleep(2e5);
  sys("xdotool mouseup 1");
  usleep(4e6);

  // -- Return to original position at far left.
  sys("xdotool keydown Alt");
  sys("xdotool keydown Down");
  usleep(2e6);
  sys("xdotool keyup Down");
  sys("xdotool keyup Alt");
  sys("xdotool keydown Alt");
  sys("xdotool keydown Left");
  usleep(3e6);
  sys("xdotool keyup Left");
  sys("xdotool keyup Alt");
}


PythonCannonDriver::PythonCannonDriver() :
  firing_(false),
  num_darts_(4)
{
  mpliBegin();
  mpli("import roslib; roslib.load_manifest('jarvis')");
  mpli("from rosrocket import *");
  mpli("rd = RocketDriver()");
}

void PythonCannonDriver::_run()
{
  while(true) {
    usleep(1e5);
    if(firing_) {
      _fire();
      firing_ = false;
    }
  }
}

void PythonCannonDriver::_fire()
{
  cout << "[PythonCannonDriver] _fire()" << endl;
  if(num_darts_ == 0) {
    cout << "[PythonCannonDriver]  Out of ammo..." << endl;
    return;
  }
  --num_darts_;

  usleep(2e5);
  mpli("rd.right(2.75)");
  usleep(2e5);
  mpli("rd.up(1)");
  usleep(2e5);
  mpli("rd.fire()");
  usleep(2e5);
  mpli("rd.down(1.5)");
  usleep(2e5);
  mpli("rd.left(3)");
  usleep(2e5);
}
