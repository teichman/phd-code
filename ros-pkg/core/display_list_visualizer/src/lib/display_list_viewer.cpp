

void display(void) {
	static int count = 0;
	glClearColor(0, 0, 0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPointSize(2);
	int start = 0, end = SCANS;
	if(ANIMATE) end = SCANS/2;
	for(int j = start; j < end; j++) {
		int i = ANIMATE ? (count + j) % SCANS : j;
		makeList(i);
		if(ready[i] > 0) // we have something to display
			glCallList(ready[i]);
	}
	count++;
}

void *graphics_thread(void *ptr) {
  param_struct_p param = (param_struct_p)ptr;
  qgui3D_initialize(param->argc, param->argv, 10, 10, WINDOW_W, WINDOW_H, 30.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
  gui3D_add_timerFunc(100, timer, 0);
  qgui3D_mainloop();
  return NULL;
}

// makes an OpenGL Display List for spin f
// ready < 1 means we want to delete an old display list and make a new one
// ready = -1 means there is initial data for a new display list
// ready = 0 means there is nothing to display
// ready > 0 means there is data in the display list for display
void makeList(int f) {
	if(ready[f] >= 0)
		return; // nothing to do here
	if(ready[f] < -1)
		glDeleteLists(-ready[f], 1); // delete old list
	GLint new_dl = glGenLists(1);
	glNewList(new_dl, GL_COMPILE);
	glBegin(GL_POINTS);
	cpoint *points = allPoints[f];
	for(int i = 0; i < pointCount[f]; i++) {
		glColor3f(points[i].r / 255.0, points[i].g / 255.0, points[i].b / 255.0);
		glVertex3f(points[i].x, points[i].y, points[i].z);
	}
	glEnd();
	glEndList();
	ready[f] = new_dl;
}

