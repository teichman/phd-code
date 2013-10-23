#include <jarvis/cannon_reactor.h>
#include <eigen_extensions/eigen_extensions.h>
#include <online_learning/dataset.h>
#include <X11/Xlib.h>  // This header causes all kinds of shit if you put it at the top.

using namespace std;
using namespace Eigen;

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

  // -- If we're sure it's a cat, fire the cannon.
  Label tpred(msg->track_prediction);
  if(tpred(cmap.toId("cat")) > 0) { 
    cout << "[CannonReactor]  Track " << msg->track_id << ": " << tpred.transpose() << std::flush;
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
  }
  cout << endl;
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
