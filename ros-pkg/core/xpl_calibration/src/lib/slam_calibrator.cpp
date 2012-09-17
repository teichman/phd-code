#include <xpl_calibration/slam_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

SlamCalibrator::SlamCalibrator(const PrimeSenseModel& model, double max_range) :
  model_(model),
  max_range_(max_range)
{
}

rgbd::Cloud::Ptr SlamCalibrator::buildMap(size_t idx, double vgsize) const
{
  ROS_ASSERT(idx < trajectories_.size());
  ROS_ASSERT(trajectories_.size() == sseqs_.size());
  ROS_DEBUG_STREAM("Building slam calibration map using max_range_ of " << max_range_);
  
  const Trajectory& traj = trajectories_[idx];
  const StreamSequence& sseq = *sseqs_[idx];

  Cloud::Ptr map(new Cloud);
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;

    cout << "Using frame " << i << " / " << traj.size() << endl;
    Frame frame;
    sseq.readFrame(i, &frame);

    HighResTimer hrt("undistort"); hrt.start();
    model_.undistort(&frame);
    hrt.stop(); cout << hrt.reportMilliseconds() << endl;
    
    Cloud::Ptr tmp(new Cloud);
    model_.frameToCloud(frame, tmp.get(), max_range_);
    pcl::transformPointCloud(*tmp, *tmp, traj.get(i).cast<float>());

    *map += *tmp;
  }

  cout << "Unfiltered map has " << map->size() << " points." << endl;
  cout << "Filtering..." << endl;
  HighResTimer hrt("filtering");
  hrt.start();
  pcl::VoxelGrid<rgbd::Point> vg;
  vg.setLeafSize(vgsize, vgsize, vgsize);
  Cloud::Ptr tmp(new Cloud);
  vg.setInputCloud(map);
  vg.filter(*tmp);
  *map = *tmp;
  hrt.stop();
  cout << hrt.reportMilliseconds() << endl;
  cout << "Filtered map has " << map->size() << " points." << endl;

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
  initial_model.resetDepthDistortionModel();
  DepthDistortionLearner ddl(initial_model);

  for(size_t i = 0; i < size(); ++i) {

    const Trajectory& traj = trajectories_[i];
    const StreamSequence& sseq = *sseqs_[i];
    rgbd::Cloud::Ptr map = buildMap(i);

    for(size_t j = 0; j < traj.size(); ++j) {
      if(!traj.exists(j))
	continue;

      // -- Add the frame.
      Frame frame;
      sseq.readFrame(j, &frame);
      ddl.addFrame(frame, map, traj.get(j).inverse());

      // -- Visualize.
      // static int num = 0;
      // if(num % 10 == 0) {
      // 	cout << "Visualizing " << j << " / " << traj.size() << endl;
      // 	cv::imshow("measurement", frame.depthImage());
      // 	Frame mapframe;
      // 	Affine3f transform = traj.get(j).inverse().cast<float>();
      // 	Cloud transformed;
      // 	pcl::transformPointCloud(*map, transformed, transform);
      // 	initial_model.cloudToFrame(transformed, &mapframe);
      // 	cv::imshow("map", mapframe.depthImage());
      // 	cv::waitKey(10);
      // }
      // ++num;
    }
  }

  HighResTimer hrt("Coverage map");
  hrt.start();
  cv::Mat3b cmap = ddl.coverageMap();
  hrt.stop();
  cout << hrt.reportMilliseconds() << endl;
  cv::imshow("coverage", cmap);
  cv::imwrite("coverage.png", cmap);
  cv::waitKey(100);
  
  cout << "Fitting models using " << ddl.size() << " frames." << endl;
  PrimeSenseModel model;
  // cout << "Fitting focal length." << endl;
  // model = ddl.fitFocalLength();
  // ddl.initial_model_ = model;
  cout << "Fitting depth distortion model." << endl;
  model = ddl.fitModel();
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
