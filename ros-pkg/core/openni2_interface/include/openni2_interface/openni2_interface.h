#ifndef OPENNI2_INTERFACE_H
#define OPENNI2_INTERFACE_H

#include <OpenNI.h>

class OpenNI2Handler
{
public:
  virtual void rgbdCallback(const openni::VideoFrameRef& color,
                            const openni::VideoFrameRef& depth) = 0;
};

class OpenNI2Interface
{
public:
  OpenNI2Interface();
  ~OpenNI2Interface();
  void setHandler(OpenNI2Handler* handler) { handler_ = handler; }
  void run();
  
private:
  OpenNI2Handler* handler_;
  openni::Device device_;
  openni::VideoStream color_stream_;
  openni::VideoFrameRef color_frame_;
  openni::VideoStream depth_stream_;
  openni::VideoFrameRef depth_frame_;

  int connect();
  void processColor();
  void processDepth();
};

#endif // OPENNI2_INTERFACE_H
