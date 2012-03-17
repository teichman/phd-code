#ifndef PROJECTOR_H
#define PROJECTOR_H

#include <rgbd_sequence/stream_sequence.h>

namespace rgbd
{

  class Frame
  {
  public:
    DepthMat depth_;
    cv::Mat3b img_;
    double timestamp_;
  };
  
  class Projector
  {
  public:
    double fx_;
    double fy_;
    double cx_;
    double cy_;

    Projector();
    void frameToCloud(const Frame& frame, Cloud* pcd) const;
    void cloudToFrame(const Cloud& pcd, Frame* frame) const;
  };

}

#endif // PROJECTOR_H
