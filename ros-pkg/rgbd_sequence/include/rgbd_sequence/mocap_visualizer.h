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
    MocapVisualizer(const TRCParser& trc,
		    const std::vector<rgbd::Cloud::Ptr>& xpl,
		    double tol);
    void run();
    
  protected:
    TRCParser trc_;
    std::vector<rgbd::Cloud::Ptr> xpl_;
    VisWrapper vw_;
    double tol_;
    int trc_idx_;
    int xpl_idx_;
    double sync_;

    Eigen::Affine3f getWorldToCameraTransform(const rgbd::Cloud& camera) const;
    void incrementXPL(int num);
    void incrementSync(double val);
    void getPointTypes(const rgbd::Cloud& frame,
		       rgbd::Cloud* camera,
		       rgbd::Cloud* checker) const;

  };

}

#endif // MOCAP_VISUALIZER_H
