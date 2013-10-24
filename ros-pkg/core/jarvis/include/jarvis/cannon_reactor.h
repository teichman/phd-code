#ifndef CANNON_REACTOR_H
#define CANNON_REACTOR_H

#include <timer/timer.h>
#include <bag_of_tricks/agent.h>
#include <online_learning/dataset.h>
#include <jarvis/reactor.h>
#include <jarvis/jarvis.h>

class CannonDriver : public Agent
{
public:
  CannonDriver();
  void fire() { firing_ = true; }
  bool firing() const { return firing_; }
  void _run();
  int ammo() const { return num_darts_; }
  
protected:
  bool firing_;
  int num_darts_;

  void sendFireMessage();
};

class CannonReactor : public Reactor
{
public:
  CannonReactor(double threshold = 10.0);
  ~CannonReactor();
  
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  double threshold_;
  HighResTimer hrt_;
  CannonDriver cannon_driver_;
  std::map<size_t, DiscreteBayesFilter> filters_;
  
  void detectionCallback(jarvis::DetectionConstPtr msg);
};

#endif // CANNON_REACTOR_H
