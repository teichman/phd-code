#include <iostream>

#include <jarvis/glut_window.h>
#include "GL/gl.h"
#include "GL/glut.h"
#include <stdexcept>

using std::cerr;

void glutLeaveMainLoop() {
  // TODO(hendrik): implement proper loop exit
  exit(-1);
}

static ViewController* vc = NULL;
static int main_window;
static int win_h()
{
  return glutGet(GLUT_WINDOW_HEIGHT);
}
static void reshape(int w, int h)
{
  vc->vp_ = Viewport(0, 0, w, h);
  glutPostRedisplay();
}

static void display()
{
//  glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  vc->display();
  glutSwapBuffers();
}

static void mouse(int b, int s, int x, int y)
{
  vc->mouse(b, s, x, win_h() - y);
  glutPostRedisplay();
}

static void motion(int x, int y)
{
  vc->motion(x, win_h() - y);
  glutPostRedisplay();
}

static void key(unsigned char k, int x, int y)
{
  if(k=='q') { fprintf(stderr, "glut_window: exiting\n"); glutLeaveMainLoop(); }
  vc->key(k, x, win_h() - y);
  glutPostRedisplay();
}

#ifndef NO_GL
static void idle()
{
  glutSetWindow(main_window);
  vc->idle();
}
#endif

void GlutWindow::PostRedisplay()
{
  glutPostRedisplay();
}

void GlutWindow::setViewController(ViewController* v)
{
  vc = v;
}

GlutWindow::GlutWindow(int argc, char* argv[]) : argc_(argc), argv_(argv)
{
  vc = this;
}


void GlutWindow::stop()
{
  glutLeaveMainLoop();
}

void GlutWindow::display()
{
  static float angle = 0;

  glViewport(vp_.x, vp_.y, vp_.w, vp_.h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-1, 1, -1, 1, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  angle++;
  glRotatef(angle, 0, 0, 1);
  glutSolidCube(.1);
}

void GlutWindow::_run() {
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowPosition(50, 50);
  glutInit(&argc_, argv_);
  glutInitWindowSize(640, 480);
//  glutSetOption(GLUT_RENDERING_CONTEXT, GLUT_USE_CURRENT_CONTEXT);
#ifdef WIN32
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
#endif
  try
  {
    main_window = glutCreateWindow(argv_[0]);
  }
  catch (std::exception &e)
  {
    cerr << "[ERROR] GLUT window could not be created.\n";
    cerr << "[ERROR] Try setting \"export LIBGL_ALWAYS_INDIRECT=1\"\n";
  }
  glutKeyboardFunc(::key);
  glutReshapeFunc(::reshape);
  glutDisplayFunc(::display);
  glutMouseFunc(::mouse);
  glutMotionFunc(::motion);
  glutIdleFunc(::idle);
  glutPostRedisplay();
  glutMainLoop();
}
