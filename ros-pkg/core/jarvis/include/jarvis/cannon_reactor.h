#ifndef REACTOR_CANNON_H
#define REACTOR_CANNON_H

#include <timer/timer.h>
#include <jarvis/reactor.h>

class CannonReactor : public Reactor
{
public:
  CannonReactor();
  ~CannonReactor();
  
protected:
  HighResTimer hrt_;
  int num_darts_;
  
  void detectionCallback(jarvis::DetectionConstPtr msg);
  void fire();
};

#endif // REACTOR_CANNON_H
