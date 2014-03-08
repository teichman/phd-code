#ifndef TRACKBALL_CLASS_H
#define TRACKBALL_CLASS_H

#include <stdio.h>
#include <jarvis/view_controller.h>

// A Trackball is an UI for complex 3D rotation via dragging the mouse
// See: http://www.opengl.org/wiki/Trackball
class Trackball : public ViewController
{
public:
  Trackball(float q0 = 0., float q1 = 0., float q2 = 0., float q3 = 1.);

  virtual void mouse(int button, int state, int x, int y);
  virtual void motion(int x, int y);

  void print();
  void clear();

protected:
  void reset();
  // returns current rotation matrix which is changed by mouse/motion
  float* matrix();
  // override if child class wants to know when matrix changed
  virtual void matrix_changed() {}

  struct State
  {
    float qcurrent[4];
    float qdelta[4];
    float xcurrent[3];
    float xdelta[3];
    int mouse_button;
    float mouse_start_x, mouse_start_y, mouse_cur_x, mouse_cur_y;
  } state;

  float m[4][4];
  float trans_z;  // changed by mouse_wheel
  bool in_motion;
};

#endif
