// -----------------------------------
// Version: Hierarchical Display lists
// Antonio Ramires Fernandes
// www.lighthouse3d.com/
// -----------------------------------
#include <GL/gl.h>
#include <GL/glut.h>
#include <stdlib.h>
#include <cstdio>
#include <math.h>
#include <iostream>

using namespace std;


int window_width = 640;
int window_height = 480;
float dist = 20;
float x_offset = 0;
float y_offset = 0;
float z_offset = 0;
float camera_fov = 30;
float min_clip_range = 0.1;
float max_clip_range = 40000.0;
int prevx = -1;
int prevy = -1;
double pan = 0;
double tilt = 0;
double move_sensitivity = 0.001;
double zoom_sensitivity = 0.2;
double min_zoom_range = 0.1;

inline double dgc_d2r(double theta) {
    return (theta * M_PI / 180.0);
}

typedef enum { IDLE, ROTATING, MOVING, ZOOMING } camera_state_t;
camera_state_t motion_state = IDLE;

void mousePress(int button, int state, int x, int y)
{
  if(state == GLUT_DOWN) {
    if(button == GLUT_LEFT_BUTTON)
      motion_state = ROTATING;
    else if(button == GLUT_MIDDLE_BUTTON)
      motion_state = MOVING;
    else if(button == GLUT_RIGHT_BUTTON)
      motion_state = ZOOMING;
  }
  else if(state == GLUT_UP)
    motion_state = IDLE;

  // if (!__gui3d_use_qt) {
  //   gui3D.modifiers = glutGetModifiers();
  // }

  // if(!(gui3D.modifiers & (GLUT_ACTIVE_CTRL | GLUT_ACTIVE_ALT))) {
  //   if(state == GLUT_DOWN) {
  //     gui3D.last_mouse_x = x;
  //     gui3D.last_mouse_y = y;
  //     if(button == GLUT_LEFT_BUTTON)
  // 	motion_state = ROTATING;
  //     else if(button == GLUT_MIDDLE_BUTTON)
  // 	motion_state = MOVING;
  //     else if(button == GLUT_RIGHT_BUTTON)
  // 	motion_state = ZOOMING;
  //   }
  //   else if(state == GLUT_UP)
  //     motion_state = IDLE;
  // }
}

void zoomCamera(double dy)
{
  dist -= dy * zoom_sensitivity * dist;
  if(dist < min_zoom_range)
    dist = min_zoom_range;
}

void moveCamera(double dx, double dy)
{
  x_offset += 
    -dy * cos(dgc_d2r(pan)) * 
    move_sensitivity * dist;
  y_offset += 
    -dy * sin(dgc_d2r(pan)) * 
    move_sensitivity * dist;
  x_offset += 
    dx * cos(dgc_d2r(pan - 90.0)) * 
    move_sensitivity * dist;
  y_offset += 
    dx * sin(dgc_d2r(pan - 90.0)) * 
    move_sensitivity * dist;
}

void rotateCamera(double dx, double dy)
{
  pan -= dx;
  tilt += dy;
  
  if(tilt < 0)
    tilt = 0;
  else if(tilt > 89.5)
    tilt = 89.5;
}

void set_display_mode_3D(int w, int h, float fovy, float near, float far)
{
  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fovy, w / (float)h, near, far);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void gui3D_switch_to_3D_mode(void)
{
  float cpan, ctilt, camera_x, camera_y, camera_z;
  
  /* setup camera view */
  cpan = pan * M_PI / 180.0;
  ctilt = tilt * M_PI / 180.0;
  camera_x = dist * cos(cpan) * cos(ctilt);
  camera_y = dist * sin(cpan) * cos(ctilt);
  camera_z = dist * sin(ctilt);
  
  set_display_mode_3D(window_width, window_height, 
                      camera_fov, min_clip_range,
                      max_clip_range);
  
  glViewport(0, 0, (GLsizei)window_width, (GLsizei)window_height);

  gluLookAt(camera_x + x_offset, 
            camera_y + y_offset,
            camera_z + z_offset, 
            x_offset, 
            y_offset,
            z_offset, 0, 0, 1);
}

void gui3D_display(void)
{
  gui3D_switch_to_3D_mode();
  glFlush();
  // if (!__gui3d_use_qt)
  //   glutSwapBuffers();
}
void mouseMotion(int x, int y)
{
  int dx, dy;

  dx = x - prevx;
  dy = y - prevy;

  if(motion_state == ROTATING) 
    rotateCamera(dx, dy);
  else if(motion_state == MOVING) 
    moveCamera(dx, dy);
  else if(motion_state == ZOOMING)
    zoomCamera(dy);

  prevx = x;
  prevy = y;
}


void passiveMouseMotion(int x, int y)
{
  prevx = x;
  prevy = y;
}



/************************************************************/





float angle=0.0;
float x=0.0f,y=1.75f,z=5.0f;
float lx=0.0f,ly=0.0f,lz=-1.0f;
float ratio=1.0;
int frame,timeaoeu,timebase=0;
char s[30];

GLuint DLid;

GLuint createDL(void);

void changeSize(int w, int h){

  // Prevent a divide by zero, when window is too short
  // (you cant make a window of zero width).
  if(h == 0)
    h = 1;

  ratio = 1.0f * w / h;
  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // Set the viewport to be the entire window
  glViewport(0, 0, w, h);

  // Set the clipping volume
  gluPerspective(45,ratio,1,1000);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(x, y, z,
	    x + lx,y + ly,z + lz,
	    0.0f,1.0f,0.0f);
}

void initScene() {

  glEnable(GL_DEPTH_TEST);
  DLid = createDL();
}


void drawSnowMan() {


  glColor3f(1.0f, 1.0f, 1.0f);

  // Draw Body
  glTranslatef(0.0f ,0.75f, 0.0f);
  glutSolidSphere(0.75f,20,20);


  // Draw Head
  glTranslatef(0.0f, 1.0f, 0.0f);
  glutSolidSphere(0.25f,20,20);

  // Draw Eyes
  glPushMatrix();
  glColor3f(0.0f,0.0f,0.0f);
  glTranslatef(0.05f, 0.10f, 0.18f);
  glutSolidSphere(0.05f,10,10);
  glTranslatef(-0.1f, 0.0f, 0.0f);
  glutSolidSphere(0.05f,10,10);
  glPopMatrix();

  // Draw Nose
  glColor3f(1.0f, 0.5f , 0.5f);
  glRotatef(0.0f,1.0f, 0.0f, 0.0f);
  glutSolidCone(0.08f,0.5f,10,2);
}



GLuint createDL() {
  GLuint snowManDL,loopDL;

  snowManDL = glGenLists(1);
  loopDL = glGenLists(1);

  glNewList(snowManDL,GL_COMPILE);
  drawSnowMan();
  glEndList();

  glNewList(loopDL,GL_COMPILE);
  for(int i = -3; i < 3; i++)
    for(int j=-3; j < 3; j++) {
      glPushMatrix();
      glTranslatef(i*10.0,0,j * 10.0);
      glCallList(snowManDL);
      glPopMatrix();
    }
  glEndList();

  return(loopDL);
}

void renderScene(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Draw ground

  glColor3f(0.9f, 0.9f, 0.9f);
  glBegin(GL_QUADS);
  glVertex3f(-100.0f, 0.0f, -100.0f);
  glVertex3f(-100.0f, 0.0f,  100.0f);
  glVertex3f( 100.0f, 0.0f,  100.0f);
  glVertex3f( 100.0f, 0.0f, -100.0f);
  glEnd();

  // Draw 36 SnowMen

  glCallList(DLid);
  frame++;
  timeaoeu=glutGet(GLUT_ELAPSED_TIME);
  if (timeaoeu - timebase > 1000) {
    sprintf(s,"FPS:%4.2f",frame*1000.0/(timeaoeu-timebase));
    glutSetWindowTitle(s);
    timebase = timeaoeu;
    frame = 0;
  }
  glutSwapBuffers();

  gui3D_display();
}

void orientMe(float ang) {


  lx = sin(ang);
  lz = -cos(ang);
  glLoadIdentity();
  gluLookAt(x, y, z,
	    x + lx,y + ly,z + lz,
	    0.0f,1.0f,0.0f);
}


void moveMeFlat(int i) {
  x = x + i*(lx)*0.1;
  z = z + i*(lz)*0.1;

  glLoadIdentity();
  gluLookAt(x, y, z,
	    x + lx,y + ly,z + lz,
	    0.0f,1.0f,0.0f);
}


void inputKey(unsigned char c, int x, int y) {

  switch (c) {
  case 'a' : angle -= 0.01f;orientMe(angle);break;
  case 's' : angle +=0.01f;orientMe(angle);break;
  case 't' : moveMeFlat(1);break;
  case 'g' : moveMeFlat(-1);break;
  }
}


/************************************************************/

int main(int argc, char **argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(100,100);
  glutInitWindowSize(300,300);
  glutCreateWindow("SnowMen");
  
  initScene();

  glutKeyboardFunc(inputKey);
  glutMotionFunc(mouseMotion);
  glutMouseFunc(mousePress);
  glutPassiveMotionFunc(passiveMouseMotion);

  glutDisplayFunc(renderScene);
  glutIdleFunc(renderScene);

  glutReshapeFunc(changeSize);

  glutMainLoop();

  return(0);
}

