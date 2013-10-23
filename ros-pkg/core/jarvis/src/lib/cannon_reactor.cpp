#include <jarvis/cannon_reactor.h>
#include <online_learning/dataset.h>
#include <X11/Xlib.h>

using namespace std;

CannonReactor::CannonReactor()
{
  // -- Start up pyrocket.
  int ret = system("killall pyrocket");
  ROS_ASSERT(ret == 0);
  ret = system("~/pyrocket/src/pyrocket &");
  ROS_ASSERT(ret == 0);

  // -- Place the mouse cursor over the fire button.
  Display* display = XOpenDisplay(NULL);
  assert(display);
  Window root = DefaultRootWindow(display);
  int row = 150;
  int col = 10;
  XWarpPointer(display, 0, root, 0, 0, 0, 0, col, row);
  XCloseDisplay(display);
    
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
  Label pred(msg->label);
  cout << "Detection: " << endl;
  cout << "  " << pred.status(cmap) << endl;

  if(hrt_.getSeconds() > 5.0) {
    fire();
    hrt_.reset();
    hrt_.start();
  }
}

void CannonReactor::fire() const
{
  cout << "[CannonReactor]  Fire!" << endl;
  int ret = system("xdotool mousedown 1");
  ROS_ASSERT(ret == 0);
  usleep(1e4);
  ret = system("xdotool mouseup 1");
  ROS_ASSERT(ret == 0);
}
