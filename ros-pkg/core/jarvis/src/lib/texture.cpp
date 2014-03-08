#include <GL/gl.h>
//#include <GL/gl_mangle.h>
#include <jarvis/texture.h>

Texture::Texture() {
  glGenTextures(1, &id_);
  SetWrap(GL_CLAMP_TO_EDGE);
  SetFilter(GL_LINEAR, GL_LINEAR);
}


Texture::Texture(int wrap, int mag_filt, int min_filt)
    : internal_format_(0), width_(0), height_(0)
{
  glGenTextures(1, &id_);
  SetWrap(wrap);
  SetFilter(mag_filt, min_filt);
}

Texture::~Texture()
{
  glDeleteTextures(1, &id_);
}

void Texture::Enable(int texture_unit)
{
  glActiveTexture(texture_unit);
  glEnable(GL_TEXTURE_2D);
  Bind();
}

void Texture::Disable(int texture_unit)
{
  glActiveTexture(texture_unit);
  glDisable(GL_TEXTURE_2D);
}

void Texture::Bind()
{
  glBindTexture(GL_TEXTURE_2D, id_);
}

void Texture::Resize(int format, int w, int h)
{
  internal_format_ = format;
  width_ = w;
  height_ = h;
}

void Texture::DrawAsQuad() {
  Enable(GL_TEXTURE0);
  glBegin(GL_QUADS);
  glTexCoord2f(0, 1); glVertex3f(0, 0, 0);
  glTexCoord2f(1, 1); glVertex3f(1, 0, 0);
  glTexCoord2f(1, 0); glVertex3f(1, 1, 0);
  glTexCoord2f(0, 0); glVertex3f(0, 1, 0);
  glEnd();
//  glActiveTexture(GL_TEXTURE0);
  glDisable(GL_TEXTURE_2D);
}

void Texture::CopyToGPU(int format, unsigned char * data, GLint level)
{
  Bind();
  glTexImage2D(GL_TEXTURE_2D, level, internal_format_, width_ >> level,
               height_ >> level, 0, format, GL_UNSIGNED_BYTE, data);
}
void Texture::CopyToGPU(int format, float * data, GLint level)
{
  Bind();
  glTexImage2D(GL_TEXTURE_2D, level, internal_format_, width_ >> level,
               height_ >> level, 0, format, GL_FLOAT, data);
}

void Texture::SetWrap(int wrap)
{
  Bind();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, wrap);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
}

void Texture::SetFilter(int mag_filt, int min_filt)
{
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filt);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filt);
}

//void Texture::ReadFromGPU(float* data, int level)
//{
//  Enable(GL_TEXTURE0);
//  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
//  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, level);
//  glGenerateMipmap(GL_TEXTURE_2D);
//  glFlush();
//  glGetTexImage(GL_TEXTURE_2D, level, GL_LUMINANCE, GL_FLOAT, data);
//  glFlush();
//  disable(GL_TEXTURE0);
//}
//


