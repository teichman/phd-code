#ifndef SLAM_CALIBRATOR_H
#define SLAM_CALIBRATOR_H

#include <pcl/filters/voxel_grid.h>
#include <rgbd_sequence/stream_sequence.h>
#include <xpl_calibration/trajectory.h>
#include <xpl_calibration/depth_distortion_learner.h>

class SlamCalibrator
{
public:
  typedef boost::shared_ptr<SlamCalibrator> Ptr;

  //! The model to use when projecting frames out into the map.
  rgbd::PrimeSenseModel model_;
  //! meters
  double max_range_;
  std::vector<Trajectory> trajectories_;
  std::vector<rgbd::StreamSequence::ConstPtr> sseqs_;

  SlamCalibrator(const rgbd::PrimeSenseModel& model, double max_range);
  rgbd::Cloud::Ptr buildMap(size_t idx, double vgsize) const;
  size_t size() const;
  rgbd::PrimeSenseModel calibrate() const;
  
protected:
};

#endif // SLAM_CALIBRATOR_H
