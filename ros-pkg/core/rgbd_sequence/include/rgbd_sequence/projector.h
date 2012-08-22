#ifndef PRIMESENSE_MODEL_H
#define PRIMESENSE_MODEL_H

#include <rgbd_sequence/stream_sequence.h>

namespace rgbd
{

  //! "Projective" point comes from the OpenNI terminology, and refers to (u, v, z), i.e.
  //! pixel id and depth value.  Here I've added color, too, so that this represents everything
  //! that is known about a pixel in an RBGD camera.
  class ProjectivePoint : public Serializable
  {
  public:
    int u_;
    int v_;
    //! In millimeters, same as the raw depth image from the primesense device.
    unsigned short z_;
    unsigned char r_;
    unsigned char g_;
    unsigned char b_;

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
  
  class PrimeSenseModel
  {
  public:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    int width_;
    int height_;

    //! Initializes with a bogus model: all params set to -1.
    PrimeSenseModel();
    void frameToCloud(const Frame& frame, Cloud* pcd) const;
    void project(const ProjectivePoint& ppt, Point* pt) const;
    // TODO
    //void project(const Point& pt, ProjectivePoint* ppt) const;
  };

}

#endif // PRIMESENSE_MODEL_H
