#include <jarvis/reactor.h>

using namespace std;

Reactor::Reactor()
{
  det_sub_ = nh_.subscribe("detections", 3, &Reactor::detectionCallback, this);
}

void Reactor::_run()
{
  ros::spin();
}

  
