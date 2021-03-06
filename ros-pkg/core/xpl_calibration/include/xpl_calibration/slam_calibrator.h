#ifndef SLAM_CALIBRATOR_H
#define SLAM_CALIBRATOR_H

#include <pcl/filters/voxel_grid.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <xpl_calibration/trajectory.h>
#include <xpl_calibration/depth_distortion_learner.h>
#include <xpl_calibration/primesense_slam.h>

class SlamCalibrator
{
public:
  typedef boost::shared_ptr<SlamCalibrator> Ptr;

  //! The model to use when projecting frames out into the map.
  rgbd::PrimeSenseModel model_;
  std::vector<Trajectory> trajectories_;
  std::vector<rgbd::StreamSequenceBase::ConstPtr> sseqs_;
  double max_range_;
  double vgsize_;

  SlamCalibrator(const rgbd::PrimeSenseModel& model, double max_range = MAX_RANGE_MAP, double vgsize = DEFAULT_VGSIZE);
  rgbd::Cloud::Ptr buildMap(size_t idx) const;
  static rgbd::Cloud::Ptr buildMap(rgbd::StreamSequenceBase::ConstPtr sseq, const Trajectory& traj, double max_range, double vgsize = DEFAULT_VGSIZE);
  size_t size() const;
  rgbd::PrimeSenseModel calibrate() const;
  DiscreteDepthDistortionModel calibrateDiscrete() const;
  double calibrateFocalLength() const;

protected:
  DepthDistortionLearner setupDepthDistortionLearner() const;
};

#endif // SLAM_CALIBRATOR_H
