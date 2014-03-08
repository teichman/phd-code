#include <math.h>
#include <jarvis/trackball.h>

static const float kTrackballSize = 0.8f;

namespace
{
  void vzero(float *v)
  {
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
  }

  void vset(float *v, float x, float y, float z)
  {
    v[0] = x;
    v[1] = y;
    v[2] = z;
  }

  void vsub(const float *src1, const float *src2, float *dst)
  {
    dst[0] = src1[0] - src2[0];
    dst[1] = src1[1] - src2[1];
    dst[2] = src1[2] - src2[2];
  }

  void vcopy(const float *v1, float *v2)
  {
    register int i;
    for (i = 0; i < 3; i++)
      v2[i] = v1[i];
  }

  void vcross(const float *v1, const float *v2, float *cross)
  {
    float temp[3];

    temp[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    temp[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    temp[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
    vcopy(temp, cross);
  }

  float vlength(const float *v)
  {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }

  void vscale(float *v, float div)
  {
    v[0] *= div;
    v[1] *= div;
    v[2] *= div;
  }

  void vnormal(float *v)
  {
    vscale(v, 1.0 / vlength(v));
  }

  float vdot(const float *v1, const float *v2)
  {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  }

  void vadd(const float *src1, const float *src2, float *dst)
  {
    dst[0] = src1[0] + src2[0];
    dst[1] = src1[1] + src2[1];
    dst[2] = src1[2] + src2[2];
  }
}

Trackball::Trackball(float q0, float q1, float q2, float q3)
    : in_motion(false)
{
  state = { { q0, q1, q2, q3 },  // current rotation
            { 0, 0, 0, 1 },  // delta rotation
            { 0, 0, 0 },
            { 0, 0, 0 }, 0, 0, 0, 0, 0 };
  clear();
}

void Trackball::print()
{
  fprintf(stderr, "q: %f %f %f %f\n", state.qcurrent[0], state.qcurrent[1],
          state.qcurrent[2], state.qcurrent[3]);
}

void Trackball::clear()
{
  reset();
  for (int i = 0; i < 4; ++i)
  {
    state.qcurrent[i] = (i == 3);
  }
  trans_z = 0;
}

void Trackball::reset()
{
  for (int i = 0; i < 4; ++i)
  {
    state.qdelta[i] = (i == 3);
  }
  state.xdelta[0] = 0;
  state.xdelta[1] = 0;
  state.xdelta[2] = 0;
  in_motion = false;
  matrix_changed();
}

// project to sphere or hyperbola
float project_to_trackball(float r, float x, float y)
{
  float d = hypot(x, y);
  if (d < r * M_SQRT1_2)
  { /* Inside sphere */
    return sqrt(r * r - d * d);
  }
  else
  { /* On hyperbola */
    float t = r / M_SQRT2;
    return t * t / d;
  }
}

void quaternion_normalize(float q[4])
{
  float d = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  for (int i = 0; i < 4; i++)
    q[i] /= d;
}

void quaternion_add(float q1[4], float q2[4], float dest[4])
{
  float t1[4], t2[4], t3[4], tf[4];

  vcopy(q1, t1);
  vscale(t1, q2[3]);

  vcopy(q2, t2);
  vscale(t2, q1[3]);

  vcross(q2, q1, t3);
  vadd(t1, t2, tf);
  vadd(t3, tf, tf);
  tf[3] = q1[3] * q2[3] - vdot(q1, q2);

  for (int i = 0; i < 4; i++)
    dest[i] = tf[i];

  static int call_count = 0;
  if (++call_count > 50)
  {
    quaternion_normalize(dest);
    call_count = 0;
  }
}

void Trackball::motion(int x, int y)
{
  if (!in_motion)
  {
    return;
  }
  x = x - vp_.x;
  y = y - vp_.y;
  state.mouse_cur_x = ((1.0f * x) / vp_.w) - 0.5f;
  state.mouse_cur_y = ((1.0f * y) / vp_.h) - 0.5f;
  if (state.mouse_button == 0)
  {
    float a[3];  // axis
    float p1[3], p2[3], d[3];

    if (state.mouse_start_x == state.mouse_cur_x
        && state.mouse_start_y == state.mouse_cur_y)
    {
      vzero(state.qdelta);
      state.qdelta[3] = 1.0;
      return;
    }

    vset(
        p1,
        state.mouse_start_x,
        state.mouse_start_y,
        project_to_trackball(kTrackballSize, state.mouse_start_x,
                             state.mouse_start_y));
    vset(
        p2,
        state.mouse_cur_x,
        state.mouse_cur_y,
        project_to_trackball(kTrackballSize, state.mouse_cur_x,
                             state.mouse_cur_y));
    vcross(p2, p1, a);
    vsub(p1, p2, d);
    float t = vlength(d) / (2.0 * kTrackballSize);

    if (t > 1.0)
      t = 1.0;
    if (t < -1.0)
      t = -1.0;
    float phi = 2.0 * asin(t);  // angle

    vnormal(a);
    vcopy(a, state.qdelta);
    vscale(state.qdelta, sin(phi / 2.0));
    state.qdelta[3] = cos(phi / 2.0);
  }
  else
  {
    state.xdelta[0] = (state.mouse_cur_x - state.mouse_start_x);
    state.xdelta[1] = (state.mouse_cur_y - state.mouse_start_y);
  }
  matrix_changed();
}

void Trackball::mouse(int button, int button_state, int x, int y)
{
  x -= vp_.x;
  y -= vp_.y;

  state.mouse_button = button;
  if (button == 0)
  {
    // mouse down
    if (button_state == 0)
    {
      if (x > 0 && y > 0 && x < vp_.w && y < vp_.h)
      {
        state.mouse_start_x = ((1.0f * x) / vp_.w) - 0.5f;
        state.mouse_start_y = ((1.0f * y) / vp_.h) - 0.5f;
        in_motion = true;
      }
    }
    else if (in_motion)
    {
      // push delta into current
      quaternion_add(state.qdelta, state.qcurrent, state.qcurrent);
      reset();  // calls matrix_changed
    }
  }
  else if (button == 3)
  {  // wheel up
    trans_z++;
    matrix_changed();
  }
  else if (button == 4)
  {  // wheel down
    trans_z--;
    matrix_changed();
  }
}

float* Trackball::matrix()
{
  static float q[4];
  quaternion_add(state.qdelta, state.qcurrent, q);

  m[0][0] = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
  m[0][1] = 2.0 * (q[0] * q[1] - q[2] * q[3]);
  m[0][2] = 2.0 * (q[2] * q[0] + q[1] * q[3]);
  m[0][3] = 0.0;

  m[1][0] = 2.0 * (q[0] * q[1] + q[2] * q[3]);
  m[1][1] = 1.0 - 2.0 * (q[2] * q[2] + q[0] * q[0]);
  m[1][2] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  m[1][3] = 0.0;

  m[2][0] = 2.0 * (q[2] * q[0] - q[1] * q[3]);
  m[2][1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  m[2][2] = 1.0 - 2.0 * (q[1] * q[1] + q[0] * q[0]);
  m[2][3] = 0.0;

  m[3][0] = 0.0;
  m[3][1] = 0.0;
  m[3][2] = 0.0;
  m[3][3] = 1.0;

  m[3][2] = .1 * trans_z;
  return (&m[0][0]);
}
