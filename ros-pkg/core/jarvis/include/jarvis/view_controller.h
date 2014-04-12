#ifndef VIEW_CONTROLLER_H
#define VIEW_CONTROLLER_H

#include <agent/lockable.h>

struct Viewport
{
  int x, y, w, h;
  inline Viewport(int x = 0, int y = 0, int w = 256, int h = 256)
      : x(x), y(y), w(w), h(h) {}

  inline bool contains(int xi, int yi)
  {
    return (xi >= x) && (yi >= y) && ((xi - x) < w) && ((yi - y) < h);
  }
};


class ViewController
{
public:
  virtual ~ViewController() {}

  // sets all context variables and draws
  virtual void display() {};
  // useful for animation
  virtual void idle() {}

  // call this when mouse goes up or down
  virtual void mouse(int button, int state, int x, int y) {}
  // call this when mouse dragged
  virtual void motion(int x, int y) {}
  virtual void key(unsigned char k, int x, int y) {}

  Viewport vp_;
};

#endif
