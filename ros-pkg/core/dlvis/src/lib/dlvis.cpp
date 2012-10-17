#include <dlvis/dlvis.h>

using namespace std;
using namespace rgbd;

DLVis::DLVis(string title) :
  title_(title),
  needs_update_(false),
  window_width_(1024),
  window_height_(768),
  dist_(10),
  x_offset_(0),
  y_offset_(0),
  z_offset_(0),
  camera_fov_(30),
  min_clip_range_(0.01),
  max_clip_range_(10000.0),
  prev_x_(-1),
  prev_y_(-1),
  pan_(0),
  tilt_(0),
  move_sensitivity_(0.001),
  zoom_sensitivity_(0.01),
  min_zoom_range_(0.1),
  motion_state_(IDLE),
  customKeyboardCallback_(NULL)
{
  ROS_ASSERT(!g_dlvis);
  g_dlvis = this;
}

void DLVis::staticDisplay()
{
  g_dlvis->display();
}

void DLVis::staticReshape(int w, int h)
{
  g_dlvis->reshape(w, h);
}

void DLVis::staticMouseMotionCallback(int x, int y)
{
  g_dlvis->mouseMotionCallback(x, y);
}

void DLVis::staticPassiveMouseMotionCallback(int x, int y)
{
  g_dlvis->passiveMouseMotionCallback(x, y);
}

void DLVis::staticMousePressCallback(int button, int state, int x, int y)
{
  g_dlvis->mousePressCallback(button, state, x, y);
}

void DLVis::staticKeyboardCallback(unsigned char c, int x, int y)
{
  g_dlvis->keyboardCallback(c, x, y);
}

void DLVis::mouseMotionCallback(int x, int y)
{
  int dx = x - prev_x_;
  int dy = y - prev_y_;
  if(motion_state_ == ROTATING) 
    rotateCamera(dx, dy);
  else if(motion_state_ == MOVING) 
    moveCamera(dx, dy);
  else if(motion_state_ == ZOOMING)
    zoomCamera(dy);

  prev_x_ = x;
  prev_y_ = y;
}

void DLVis::passiveMouseMotionCallback(int x, int y)
{
  prev_x_ = x;
  prev_y_ = y;
}

void DLVis::mousePressCallback(int button, int state, int x, int y)
{
  if(state == GLUT_DOWN) {
    if(button == GLUT_LEFT_BUTTON)
      motion_state_ = ROTATING;
    else if(button == GLUT_MIDDLE_BUTTON)
      motion_state_ = MOVING;
    else if(button == GLUT_RIGHT_BUTTON)
      motion_state_ = ZOOMING;
  }
  else if(state == GLUT_UP)
    motion_state_ = IDLE;
}

void DLVis::keyboardCallback(unsigned char c, int x, int y)
{
  if(customKeyboardCallback_)
    customKeyboardCallback_(c, x, y);
}

void DLVis::zoomCamera(double dy)
{
  dist_ += dy * zoom_sensitivity_ * dist_;
  if(dist_ < min_zoom_range_)
    dist_ = min_zoom_range_;
}

inline double d2r(double theta)
{
  return (theta * M_PI / 180.0);
}

void DLVis::moveCamera(double dx, double dy)
{
  x_offset_ += 
    -dy * cos(d2r(pan_)) * 
    move_sensitivity_ * dist_;
  y_offset_ += 
    -dy * sin(d2r(pan_)) * 
    move_sensitivity_ * dist_;
  x_offset_ += 
    dx * cos(d2r(pan_ - 90.0)) * 
    move_sensitivity_ * dist_;
  y_offset_ += 
    dx * sin(d2r(pan_ - 90.0)) * 
    move_sensitivity_ * dist_;
}

void DLVis::rotateCamera(double dx, double dy)
{
  pan_ -= dx;
  tilt_ += dy;
  
  if(tilt_ < -89.5)
    tilt_ = -89.5;
  else if(tilt_ > 89.5)
    tilt_ = 89.5;
}

void DLVis::display()
{
  //ScopedTimer st("DLVis::display()");
  
  // -- Update the display list if necessary.
  lockWrite();
  if(needs_update_)
    updateDisplayList();
  needs_update_ = false;
  unlockWrite();

  glClearColor(0, 0, 0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPointSize(2);
  if(cloud_)
    glCallList(dlid_);
  glutSwapBuffers();
  
  // -- Set the camera view.
  float cpan, ctilt, camera_x, camera_y, camera_z;
  cpan = pan_ * M_PI / 180.0;
  ctilt = tilt_ * M_PI / 180.0;
  camera_x = dist_ * cos(cpan) * cos(ctilt);
  camera_y = dist_ * sin(cpan) * cos(ctilt);
  camera_z = dist_ * sin(ctilt);
  
  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(camera_fov_, window_width_ / (float)window_height_, min_clip_range_, max_clip_range_);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glViewport(0, 0, (GLsizei)window_width_, (GLsizei)window_height_);

  gluLookAt(camera_x + x_offset_, 
            camera_y + y_offset_,
            camera_z + z_offset_, 
            x_offset_, 
            y_offset_,
            z_offset_, 0, 0, 1);

  glFlush();
}

void DLVis::reshape(int w, int h)
{
  // Prevent a divide by zero, when window is too short
  // (you cant make a window of zero width).
  if(h == 0)
    h = 1;

  float ratio = 1.0f * w / h;
  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // Set the viewport to be the entire window
  glViewport(0, 0, w, h);

  // Set the clipping volume
  //gluPerspective(45,ratio,1,1000);
  // glMatrixMode(GL_MODELVIEW);
  // glLoadIdentity();
  // gluLookAt(0, 1.75, 5,
  // 	    0, 1.75, 4,
  // 	    0.0f, 1.0f, 0.0f);
}

void DLVis::setPointCloud(rgbd::Cloud::ConstPtr cloud)
{
  scopeLockWrite;
  cloud_ = cloud;
  needs_update_ = true;
}

void DLVis::_run()
{
  int argc = 1;
  char* argv[argc];
  argv[0] = (char*)string("aoeu").c_str();
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(window_width_, window_height_);
  glutCreateWindow(title_.c_str());
  
  glEnable(GL_DEPTH_TEST);

  glutKeyboardFunc(staticKeyboardCallback);
  glutMotionFunc(staticMouseMotionCallback);
  glutMouseFunc(staticMousePressCallback);
  glutPassiveMotionFunc(staticPassiveMouseMotionCallback);
  glutDisplayFunc(staticDisplay);
  glutIdleFunc(staticDisplay);
  glutReshapeFunc(staticReshape);

  glutMainLoop();
}

void DLVis::updateDisplayList()
{  
  // Fill dlid_ with the cloud display list.
  ROS_WARN("DLVis needs to delete old lists.");
  //glDeleteLists(-ready[f], 1); // delete old list
  if(!cloud_)
    return;

  const Cloud& pcd = *cloud_;
  cout << "Updating display list with " << pcd.size() << " points." << endl;

  dlid_ = glGenLists(1);
  glNewList(dlid_, GL_COMPILE);
  glBegin(GL_POINTS);
  for(size_t i = 0; i < pcd.size(); ++i) { 
    glColor3f(pcd[i].r / 255.0, pcd[i].g / 255.0, pcd[i].b / 255.0);
    glVertex3f(pcd[i].x, pcd[i].y, pcd[i].z);
  }
  glEnd();
  glEndList();    
}

void DLVis::registerKeyboardCallback(KeyboardCallbackFunction func)
{
  customKeyboardCallback_ = func;
}
