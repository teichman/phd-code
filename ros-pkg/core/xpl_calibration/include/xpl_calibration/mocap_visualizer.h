#ifndef MOCAP_VISUALIZER_H
#define MOCAP_VISUALIZER_H

#include <pcl/common/transforms.h>
#include <rgbd_sequence/vis_wrapper.h>
#include <xpl_calibration/trc_parser.h>
#include <xpl_calibration/mocap_detector.h>

// XPL has 3-ball marker on it with distances of 11.5 cm and 13.25 cm from one ball to the two others.

class MocapVisualizer
{
public:
  MocapVisualizer(const TRCParser& trc,
                  rgbd::StreamSequenceBase::ConstPtr sseq,
                  double tol = 0.0075);
  void run();
    
protected:
  TRCParser trc_;
  rgbd::StreamSequenceBase::ConstPtr sseq_;
  rgbd::VisWrapper vw_;
  double tol_;
  int trc_idx_;
  int xpl_idx_;
  double sync_;
  double sseq_start_;

  Eigen::Affine3f getWorldToCameraTransform(const rgbd::Cloud& camera) const;
  void incrementXPL(int num);
  void incrementSync(double val);
  void getTRCPoints(const rgbd::Cloud& frame,
                    rgbd::Cloud* camera,
                    rgbd::Cloud* checker) const;
  void getXPLPoints(rgbd::Cloud* xpl) const;

};

#endif // MOCAP_VISUALIZER_H
