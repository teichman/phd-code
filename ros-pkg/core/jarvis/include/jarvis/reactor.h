#ifndef REACTOR_H
#define REACTOR_H

#include <ros/ros.h>
#include <jarvis/Detection.h>
#include <agent/agent.h>

class Reactor : public Agent
{
public:
  Reactor();
  virtual ~Reactor() {}
  void _run();
  virtual void detectionCallback(jarvis::DetectionConstPtr msg) = 0;
  
protected:
  ros::NodeHandle nh_;
  ros::Subscriber det_sub_;
};

#endif // REACTOR_H
