#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include <opencv2/core/core.hpp>
#include <openni2_interface/synchronizer.h>
#include <sentinel/Foreground.h>
#include <sentinel/Background.h>

class Reconstructor
{
public:
  //! The reconstruction.
  cv::Mat3b img_;
  
  Reconstructor() : sync_(0.01) {}
  void update(sentinel::ForegroundConstPtr fgmsg);
  void update(sentinel::BackgroundConstPtr bgmsg);
  cv::Mat3b stylizedImage(int scale = 1) const;
  
protected:
  Synchronizer<sentinel::ForegroundConstPtr, sentinel::BackgroundConstPtr> sync_;

  void process();
  void update(sentinel::ForegroundConstPtr fgmsg, sentinel::BackgroundConstPtr bgmsg);
};

#endif // RECONSTRUCTOR_H
