#include <xpl_calibration/slam_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;


SlamCalibrator::SlamCalibrator() :
  max_range_(3.5)
{
}

rgbd::Cloud::Ptr SlamCalibrator::buildMap(size_t idx) const
{
  return buildMap(idx, sseqs_[idx]->model_);
}

rgbd::Cloud::Ptr SlamCalibrator::buildMap(size_t idx, const rgbd::PrimeSenseModel& model) const
{
  pcl::VoxelGrid<rgbd::Point> vg;
  vg.setLeafSize(0.02, 0.02, 0.02);

  const Trajectory& traj = trajectories_[idx];
  const StreamSequence& sseq = *sseqs_[idx];

  Cloud::Ptr map(new Cloud);
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;

    Frame frame;
    sseq.readFrame(i, &frame);
    Cloud::Ptr tmp(new Cloud);
    model.frameToCloud(frame, tmp.get(), max_range_);
    pcl::transformPointCloud(*tmp, *tmp, traj.get(i).cast<float>());

    *map += *tmp;
    vg.setInputCloud(map);
    vg.filter(*tmp);
    *map = *tmp;
  }

  return map;
}
