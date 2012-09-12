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

  ROS_ASSERT(idx < trajectories_.size());
  ROS_ASSERT(trajectories_.size() == sseqs_.size());
  
  const Trajectory& traj = trajectories_[idx];
  const StreamSequence& sseq = *sseqs_[idx];

  Cloud::Ptr map(new Cloud);
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;

    cout << "Using frame " << i << " / " << traj.size() << endl;
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

size_t SlamCalibrator::size() const
{
  ROS_ASSERT(trajectories_.size() == sseqs_.size());
  return trajectories_.size();
}

PrimeSenseModel SlamCalibrator::calibrate() const
{
  PrimeSenseModel initial_model = sseqs_[0]->model_;
  DepthDistortionLearner learner(initial_model);
  
  for(size_t i = 0; i < size(); ++i) {
    const Trajectory& traj = trajectories_[i];
    const StreamSequence& sseq = *sseqs_[i];
    rgbd::Cloud::Ptr map = buildMap(i);

    for(size_t j = 0; j < traj.size(); ++j) {
      if(!traj.exists(j))
	continue;

      Affine3f transform = traj.get(j).inverse().cast<float>();
      Frame frame;
      sseq.readFrame(j, &frame);
      learner.addFrame(frame, map, transform);

      // -- Visualize.
      // cout << "Visualizing " << j << " / " << traj.size() << endl;
      // cv::imshow("measurement", frame.depthImage());
      // rgbd::Cloud transformed;
      // pcl::transformPointCloud(*map, transformed, transform);
      // Frame mapframe;
      // initial_model.cloudToFrame(transformed, &mapframe);
      // cv::imshow("map", mapframe.depthImage());
      // cv::waitKey(300);
    }
  }

  PrimeSenseModel model = learner.fitModel();
  cout << "== Initial model: " << endl;
  cout << sseqs_[0]->model_.status();
  cout << "== Final model: " << endl;
  cout << model.status();
  
  MatrixXd wcat(model.weights_.rows(), 2);
  wcat.col(0) = sseqs_[0]->model_.weights_;
  wcat.col(1) = model.weights_;
  cout << wcat << endl;

  return model;
}
