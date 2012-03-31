#ifndef MOCAP_VISUALIZER_H
#define MOCAP_VISUALIZER_H

#include <pcl/common/transforms.h>
#include <rgbd_sequence/trc_parser.h>
#include <rgbd_sequence/vis_wrapper.h>

namespace rgbd
{

  class MocapVisualizer
  {
  public:
    MocapVisualizer(const TRCParser& trc, double tol);
    void run();
    
  protected:
    TRCParser trc_;
    VisWrapper vw_;
    double tol_;

    Eigen::Affine3f getWorldToCameraTransform(const std::vector<rgbd::Point>& camera) const;
  };

}

#endif // MOCAP_VISUALIZER_H
