#include <jarvis/reactor.h>

using namespace std;

Reactor::Reactor()
{
  // Keep an infinite queue of detection messages if necessary.
  det_sub_ = nh_.subscribe("detections", 0, &Reactor::detectionCallback, this);
}

void Reactor::_run()
{
  ros::spin();
}

  
