#include <xpl_calibration/slam_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

SlamCalibrator::SlamCalibrator(const PrimeSenseModel& model, double max_range, double vgsize) :
  model_(model),
  max_range_(max_range),
  vgsize_(vgsize)
{
}

rgbd::Cloud::Ptr SlamCalibrator::buildMap(StreamSequenceBase::ConstPtr sseq, const Trajectory& traj, double max_range, double vgsize)
{
  ROS_DEBUG_STREAM("Building slam calibration map using max range of " << max_range);
  ROS_WARN("SlamCalibrator::buildMap does not use distortion model.");
  
  Cloud::Ptr map(new Cloud);
  int num_used_frames = 0;
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;

    cout << "Using frame " << i << " / " << traj.size() << endl;
    Frame frame;
    sseq->readFrame(i, &frame);
    
    Cloud::Ptr tmp(new Cloud);
    sseq->model_.frameToCloud(frame, tmp.get(), max_range);
    Cloud::Ptr nonans(new Cloud);
    nonans->reserve(tmp->size());
    for(size_t j = 0; j < tmp->size(); ++j)
      if(isFinite(tmp->at(j)))
        nonans->push_back(tmp->at(j));
        
    pcl::transformPointCloud(*nonans, *nonans, traj.get(i).cast<float>());

    *map += *nonans;
    ++num_used_frames;
    // Added intermediate filtering to handle memory overload on huge maps
    if(num_used_frames % 50 == 0)
    {
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
    }
  }

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

rgbd::Cloud::Ptr SlamCalibrator::buildMap(size_t idx) const
{
  ROS_ASSERT(idx < trajectories_.size());
  ROS_ASSERT(trajectories_.size() == sseqs_.size());
  return buildMap(sseqs_[idx], trajectories_[idx], max_range_, vgsize_);
}

size_t SlamCalibrator::size() const
{
  ROS_ASSERT(trajectories_.size() == sseqs_.size());
  return trajectories_.size();
}

DiscreteDepthDistortionModel SlamCalibrator::calibrateDiscrete() const
{
  DepthDistortionLearner ddl = setupDepthDistortionLearner();
  return ddl.fitDiscreteModel();
}

DepthDistortionLearner SlamCalibrator::setupDepthDistortionLearner() const
{
  PrimeSenseModel initial_model = sseqs_[0]->model_;
  initial_model.resetDepthDistortionModel();
  DepthDistortionLearner ddl(initial_model);

  for(size_t i = 0; i < size(); ++i) {

    const Trajectory& traj = trajectories_[i];
    StreamSequenceBase::ConstPtr sseq = sseqs_[i];
    rgbd::Cloud::Ptr map = buildMap(i);

    for(size_t j = 0; j < traj.size(); ++j) {
      if(!traj.exists(j))
        continue;

      // -- Add the frame.
      Frame frame;
      sseq->readFrame(j, &frame);
      ddl.addFrame(frame, map, traj.get(j).inverse());

      // -- Visualize.
      // static int num = 0;
      // if(num % 10 == 0) {
      //         cout << "Visualizing " << j << " / " << traj.size() << endl;
      //         cv::imshow("measurement", frame.depthImage());
      //         Frame mapframe;
      //         Affine3f transform = traj.get(j).inverse().cast<float>();
      //         Cloud transformed;
      //         pcl::transformPointCloud(*map, transformed, transform);
      //         initial_model.cloudToFrame(transformed, &mapframe);
      //         cv::imshow("map", mapframe.depthImage());
      //         cv::waitKey(10);
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

  return ddl;
}

double SlamCalibrator::calibrateFocalLength() const
{
  DepthDistortionLearner ddl = setupDepthDistortionLearner();
  cout << "Fitting focal length using " << ddl.size() << " frames." << endl;
  PrimeSenseModel model = ddl.fitFocalLength();
  ROS_ASSERT(model.fx_ == model.fy_);
  return model.fx_;
}

PrimeSenseModel SlamCalibrator::calibrate() const
{
  DepthDistortionLearner ddl = setupDepthDistortionLearner();
    
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
