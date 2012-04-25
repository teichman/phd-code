#ifndef PROJECTOR_H
#define PROJECTOR_H

#include <rgbd_sequence/stream_sequence.h>

namespace rgbd
{

  class ProjectedPoint : public Serializable
  {
  public:
    int u_;
    int v_;
    unsigned short z_;

    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  };
  
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
    int width_;
    int height_;

    Projector();
    void frameToCloud(const Frame& frame, Cloud* pcd) const;
    //! Doesn't project.  Just takes u, v values as they are.
    //! TODO: This should use the intrinsics.
    void cloudToFrame(const Cloud& pcd, Frame* frame) const;
    void project(const ProjectedPoint& ppt, Point* pt) const;
    void project(const Point& pt, ProjectedPoint* ppt) const;
    static void project(double fx, double fy, double cx, double cy,
			const ProjectedPoint& ppt, Point* pt);    
  };

}

#endif // PROJECTOR_H
