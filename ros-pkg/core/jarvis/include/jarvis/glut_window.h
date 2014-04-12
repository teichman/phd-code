#ifndef GLUT_WINDOW_H
#define GLUT_WINDOW_H

#include <jarvis/trackball.h>
#include <agent/agent.h>

class GlutWindow : public Trackball, public Agent
{
public:
  GlutWindow(int argc, char* argv[]);
  void setViewController(ViewController* v);

  // end display loop
  void stop();

  // some default stuff
  virtual void key(unsigned char k, int x, int y) {}
  virtual void display();

  virtual void _run();
private:
  int argc_;
  char **argv_;
};

#endif
