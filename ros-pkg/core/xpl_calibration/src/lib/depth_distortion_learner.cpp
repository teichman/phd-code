#include <xpl_calibration/depth_distortion_learner.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

DepthDistortionLearner::DepthDistortionLearner(const PrimeSenseModel& initial_model) :
  initial_model_(initial_model)
{
  // -- Make sure the initial model is reasonable.
  //    It's going to be used for projection assuming no depth distortion.
  ROS_ASSERT(!initial_model_.hasDepthDistortionModel());
}

void DepthDistortionLearner::addFrame(Frame frame,
				      rgbd::Cloud::ConstPtr pcd,
				      const Eigen::Affine3f& transform)
{
  frames_.push_back(frame);
  pcds_.push_back(pcd);
  transforms_.push_back(transform);
}

PrimeSenseModel DepthDistortionLearner::fitModel() const
{
  // -- Accumulate statistics.
  statistics_.clear();
  statistics_.resize(initial_model_.height_, vector<PixelStats>(initial_model_.width_));
  for(size_t i = 0; i < statistics_.size(); ++i)
    for(size_t j = 0; j < statistics_[i].size(); ++j)
      statistics_[i][j].reserve(frames_.size());
  
  double min_mult = 0.85;
  double max_mult = 1.25;
  Cloud::Ptr transformed(new Cloud);
  ROS_WARN("Model fitting currently assumes that both clouds are sparse.  This is not the case for SLAM.  It needs to reason about occlusion for that to work.");
  for(size_t i = 0; i < frames_.size(); ++i) {
    pcl::transformPointCloud(*pcds_[i], *transformed, transforms_[i]);
    for(size_t j = 0; j < transformed->size(); ++j) {
      if(!isFinite(transformed->at(j)))
	continue;

      ProjectivePoint pp;
      initial_model_.project(transformed->at(j), &pp);
      if(pp.u_ < 0 || pp.u_ >= proj.width_ || pp.v_ < 0 || pp.v_ >= proj.height_)
	continue;

      pp.z_ = (*frames_[i].depth_)(pp.v_, pp.u_);
      if(pp.z_ == 0)
	continue;

      Point 3dpt;
      initial_model_.project(pp, &3dpt);
      double reported_range = 3dpt.getVector3fMap().norm();
      double ground_truth_range = transformed->at(j).getVector3fMap().norm();
      if(isinf(reported_range) || isnan(reported_range) || isinf(ground_truth_range) || isnan(ground_truth_range)) {
	ROS_WARN("Something unexpected is happening in DepthDistortionLearner.");
	continue;
      }
      
      // If the range is completely off, assume it's due to misalignment and not distortion.
      double mult = ground_truth_range / reported_range;
      if(mult < min_mult || mult > max_mult)
	continue;

      // Finally, if we passed all the tests, add this as a training example for this pixel.
      statistics_[pp.v_][pp.u_].addPoint(ground_truth_range, reported_range);
    }
  }

  int num_tr_ex = 0;
  for(size_t y = 0; y < statistics_.size(); ++y)
    for(size_t x = 0; x < statistics_[y].size(); ++x)
      for(size_t i = 0; i < statistics_[y][x].asus_.size(); ++i)
	++num_tr_ex;
  cout << "Fitting depth distortion model with " << num_tr_ex << " training examples." << endl;

  // -- Assemble dataset.
  int num_features = PrimeSenseModel::numFeatures();
  MatrixXd X(num_features, num_tr_ex);
  VectorXd Y(num_tr_ex);

  int idx = 0;
  VectorXd us(4);
  VectorXd vs(4);
  VectorXd ms(4);
  VectorXd measurements(num_tr_ex);
  ProjectivePoint ppt;
  for(size_t y = 0; y < statistics_.size(); ++y) {
    for(size_t x = 0; x < statistics_[y].size(); ++x) {
      for(size_t i = 0; i < statistics_[y][x].asus_.size(); ++i, ++idx) { 
	ppt.u_ = x;
	ppt.v_ = y;
	ppt.z_ = statistics_[y][x].asus_[i];
	X.col(idx) = PrimeSenseModel::computeFeatures(ppt);
	Y(idx) = statistics_[y][x].velo_[i];
	measurements(idx) = statistics_[y][x].asus_[i];
      }
    }
  }

  // -- Fit the model.
  PrimeSenseModel model = initial_model_;
  MatrixXd xxt = X * X.transpose();
  ROS_ASSERT(xxt.rows() == PrimeSenseModel::numFeatures());
  VectorXd b = X*Y;
  model.weights_ = xxt.ldlt().solve(b);
  cout << "Weights: " << model.weights_.transpose() << endl;
  
  VectorXd pre_differences = measurements - Y;
  double pre_obj = pre_differences.array().pow(2).sum() / (double)Y.rows();
  cout << "Mean error before fitting model: " << pre_obj << endl;

  VectorXd differences = (model.weights_.transpose() * X).transpose() - Y;
  double obj = differences.array().pow(2).sum() / (double)Y.rows();
  cout << "Mean error after fitting model: " << obj << endl;

  return model;
}

