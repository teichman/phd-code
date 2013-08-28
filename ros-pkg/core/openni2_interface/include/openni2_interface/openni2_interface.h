#ifndef OPENNI2_INTERFACE_H
#define OPENNI2_INTERFACE_H

#include <iostream>
#include <OpenNI.h>
#include <openni2_interface/synchronizer.h>

class OpenNI2Handler
{
public:
  virtual ~OpenNI2Handler()
  {
#if JARVIS_DEBUG
    std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif
  }
  virtual void rgbdCallback(const openni::VideoFrameRef& color,
                            const openni::VideoFrameRef& depth) = 0;
};

class OpenNI2Interface
{
public:
  enum Resolution { VGA = 0, QVGA = 1 };
  
  OpenNI2Interface(Resolution resolution);
  //! Take care to call the destructor on shut down.
  //! Starting OpenNI the next time can be annoying if you
  //! don't shut it down properly.
  ~OpenNI2Interface();
  void setHandler(OpenNI2Handler* handler) { handler_ = handler; }
  void run();
  void terminate() { terminating_ = true; }
  
private:
  Resolution resolution_;
  OpenNI2Handler* handler_;
  openni::Device device_;
  openni::VideoStream color_stream_;
  openni::VideoStream depth_stream_;
  //! color, depth.
  Synchronizer<openni::VideoFrameRef, openni::VideoFrameRef> sync_;
  bool terminating_;
  
  int connect();
  void processColor();
  void processDepth();
  void processSynchronized();
};

#endif // OPENNI2_INTERFACE_H
