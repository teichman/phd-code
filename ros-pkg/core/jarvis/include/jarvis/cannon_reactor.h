#ifndef CANNON_REACTOR_H
#define CANNON_REACTOR_H

#include <timer/timer.h>
#include <agent/agent.h>
#include <online_learning/dataset.h>
#include <jarvis/reactor.h>
#include <jarvis/jarvis.h>

class GUICannonDriver : public Agent
{
public:
  GUICannonDriver();
  void fire() { firing_ = true; }
  bool firing() const { return firing_; }
  void _run();
  int ammo() const { return num_darts_; }
  
protected:
  bool firing_;
  int num_darts_;

  void sendFireMessage();
};

class PythonCannonDriver : public Agent
{
public:
  PythonCannonDriver();
  void fire() { firing_ = true; }
  bool firing() const { return firing_; }
  int ammo() const { return num_darts_; }

protected:
  bool firing_;
  int num_darts_;

  void _run();
  void _fire();
};

class CannonReactor : public Reactor
{
public:
  CannonReactor(size_t min_num_frames = 30, double threshold = 3.0);
  ~CannonReactor();
  
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  size_t min_num_frames_;
  double threshold_;
  HighResTimer hrt_;
  PythonCannonDriver cannon_driver_;
  std::map<size_t, DiscreteBayesFilter> filters_;
  
  void detectionCallback(jarvis::DetectionConstPtr msg);
};

#endif // CANNON_REACTOR_H
