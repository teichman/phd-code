#ifndef DLVIS_H
#define DLVIS_H

#include <GL/gl.h>
#include <GL/glut.h>
#include <agent/agent.h>
#include <timer/timer.h>
#include <rgbd_sequence/primesense_model.h>

class DLVis;
DLVis* g_dlvis = NULL;  // There can be only one.

//! Display list visualizer.
class DLVis : public Agent
{
public:
  typedef enum { IDLE, ROTATING, MOVING, ZOOMING } camera_state_t;
  typedef void (*KeyboardCallbackFunction)(unsigned char, int, int);
  
  DLVis(std::string title = "DLVis");
  void setPointCloud(rgbd::Cloud::ConstPtr cloud);
  void registerKeyboardCallback(KeyboardCallbackFunction func);
  
protected:
  std::string title_;
  rgbd::Cloud::ConstPtr cloud_;
  bool needs_update_;
  int window_width_;
  int window_height_;
  float dist_;
  float x_offset_;
  float y_offset_;
  float z_offset_;
  float camera_fov_;
  float min_clip_range_;
  float max_clip_range_;
  int prev_x_;
  int prev_y_;
  double pan_;
  double tilt_;
  double move_sensitivity_;
  double zoom_sensitivity_;
  double min_zoom_range_;
  camera_state_t motion_state_;
  GLuint dlid_;
  ThreadPtr glut_thread_;
  KeyboardCallbackFunction customKeyboardCallback_;

  void zoomCamera(double dy);
  void moveCamera(double dx, double dy);
  void rotateCamera(double dx, double dy);

  static void staticMouseMotionCallback(int x, int y);
  static void staticPassiveMouseMotionCallback(int x, int y);
  static void staticMousePressCallback(int button, int state, int x, int y);
  static void staticKeyboardCallback(unsigned char c, int x, int y);
  void mouseMotionCallback(int x, int y);
  void passiveMouseMotionCallback(int x, int y);
  void mousePressCallback(int button, int state, int x, int y);
  void keyboardCallback(unsigned char c, int x, int y);
  
  void display();
  static void staticDisplay();
  void reshape(int w, int h);
  static void staticReshape(int w, int h);
  void _run();
  void updateDisplayList();
  
};

#endif // DLVIS_H
