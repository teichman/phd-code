#ifndef __XPL_CALIBRATION__MOCAP_DETECTOR_H__
#define __XPL_CALIBRATION__MOCAP_DETECTOR_H__

#include <rgbd_sequence/stream_sequence.h>
#include <pcl/visualization/cloud_viewer.h>

using rgbd::Point;
using rgbd::Cloud;
using rgbd::StreamSequence;

class MocapDetector
{
  public:
    MocapDetector(int checker_cols=6, int checker_rows=8, float square_size=0.1016);
    
    bool locatePoints(const StreamSequence::ConstPtr &seq, size_t frame, Point &tl, Point &tr, Point& bl, Point &br)
      const;

  private:
    int checker_cols_, checker_rows_;
    Eigen::Vector3f tl_, tr_, bl_, br_; //Mocap markers in world frame
    float square_size_;

};

#endif //__XPL_CALIBRATION__MOCAP_DETECTOR_H__
