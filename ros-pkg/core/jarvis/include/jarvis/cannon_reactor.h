#ifndef REACTOR_CANNON_H
#define REACTOR_CANNON_H

#include <jarvis/reactor.h>

class CannonReactor : public Reactor
{
public:
  CannonReactor();
  void detectionCallback(jarvis::DetectionConstPtr msg);
};

#endif // REACTOR_CANNON_H
