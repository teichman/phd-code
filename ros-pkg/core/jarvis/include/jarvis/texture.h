#ifndef TEXTURE_H
#define TEXTURE_H

#include <boost/shared_ptr.hpp>

class Texture
{
public:
  boost::shared_ptr<Texture> Ptr;
  Texture();
  Texture(int wrap, int mag_filt, int min_filt);
  ~Texture();

  void Enable(int texture_unit);
  void Disable(int texture_unit);
  void Bind();
  void Resize(int format, int w, int h);
  void DrawAsQuad();

  // level isLOD for mipmapped textures
  void CopyToGPU(int format, unsigned char* data, int level = 0);
  void CopyToGPU(int format, float* data, int level = 0);
//  void ReadFromGPU(float *data, int level = 0);

  void SetWrap(int wrap);
  void SetFilter(int mag_filt, int min_filt);

protected:
  unsigned int id_;  // OpenGL identifier
  int internal_format_;
  int width_, height_;
};

#endif
