#include <jarvis/cannon_reactor.h>
#include <online_learning/dataset.h>
#include <X11/Xlib.h>

using namespace std;

CannonReactor::CannonReactor() :
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
    
  hrt_.start();

  // -- Put the cannon in an initial known position at far left and pointed down.
  usleep(1e6);
  ret = system("xdotool keydown Alt; xdotool keydown Down; sleep 3; xdotool keyup Down; xdotool keyup Alt");
  ROS_ASSERT(ret == 0);
  ret = system("xdotool keydown Alt; xdotool keydown Left; sleep 5; xdotool keyup Left; xdotool keyup Alt");
  ROS_ASSERT(ret == 0);
}

CannonReactor::~CannonReactor()
{
  int ret = system("killall pyrocket");
  ROS_ASSERT(ret == 0);
}

void CannonReactor::detectionCallback(jarvis::DetectionConstPtr msg)
{
  NameMapping cmap(msg->cmap);
  Label pred(msg->track_prediction);
  // cout << "Detection: " << endl;
  // cout << "  " << pred.status(cmap) << endl;

  if(!cmap.hasName("cat")) {
    ROS_WARN_ONCE("CannonReactor expects detections messages that make predictions about cats.");
    return;
  }
  
  if(pred(cmap.toId("cat")) > 1.0 && hrt_.getSeconds() > 7.0) {
    fire();
    hrt_.reset();
    hrt_.start();
  }
}

void CannonReactor::fire()
{
  if(num_darts_ == 0) {
    cout << "[CannonReactor]  Out of ammo..." << endl;
    return;
  }
  --num_darts_;
  
  cout << "[CannonReactor]  Fire!" << endl;
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
