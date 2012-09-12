#include <xpl_calibration/depth_distortion_learner.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

DepthDistortionLearner::DepthDistortionLearner(const PrimeSenseModel& initial_model) :
  initial_model_(initial_model)
{
  // -- Make sure the initial model is reasonable.
  //    It's going to be used for projection assuming no depth distortion.
  if(initial_model_.hasDepthDistortionModel()) {
    ROS_FATAL("DepthDistortionLearner should not have an initial_model_ with a depth distortion model already built in to it.");
    ROS_FATAL_STREAM(initial_model_.weights_);
    abort();
  }
}

void DepthDistortionLearner::addFrame(Frame frame,
				      rgbd::Cloud::ConstPtr pcd,
				      const Eigen::Affine3f& transform)
{
  frames_.push_back(frame);
  pcds_.push_back(pcd);
  transforms_.push_back(transform);
}

PrimeSenseModel DepthDistortionLearner::fitModel()
{
  ROS_ASSERT(frames_.size() == pcds_.size());
  ROS_ASSERT(frames_.size() == transforms_.size());

  double min_mult = 0.85;
  double max_mult = 1.25;
  vector<VectorXd> xvec;
  vector<double> yvec;
  vector<double> mvec;
  xvec.reserve(1e6);
  yvec.reserve(1e6);
  mvec.reserve(1e6);
  
  Cloud transformed;
  for(size_t i = 0; i < frames_.size(); ++i) {
    pcl::transformPointCloud(*pcds_[i], transformed, transforms_[i]);
    Frame mapframe;
    initial_model_.cloudToFrame(transformed, &mapframe);
    const DepthMat& depth = *frames_[i].depth_;
    const DepthMat& mapdepth = *mapframe.depth_;

    ProjectivePoint ppt;
    Point pt;
    for(ppt.v_ = 0; ppt.v_ < depth.rows(); ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < depth.cols(); ++ppt.u_) {
	if(mapdepth(ppt.v_, ppt.u_) == 0 || depth(ppt.v_, ppt.u_) == 0)
	  continue;

	if(rand() % 5 != 0)
	  continue;
	
	ppt.z_ = mapdepth(ppt.v_, ppt.u_);
	initial_model_.project(ppt, &pt);
	double mapdist = pt.getVector3fMap().norm();
	ppt.z_ = depth(ppt.v_, ppt.u_);
	initial_model_.project(ppt, &pt);
	double measdist = pt.getVector3fMap().norm();
		
	// If the range is completely off, assume it's due to misalignment and not distortion.
	double mult = mapdist / measdist;
	if(mult > max_mult || mult < min_mult)
	  continue;

	xvec.push_back(initial_model_.computeFeatures(ppt));
	yvec.push_back(mult);
	mvec.push_back(measdist);
      }
    }
  }	

  cout << "Fitting depth distortion model with " << xvec.size() << " training examples." << endl;

  // -- Assemble dataset.
  int num_features = initial_model_.numFeatures();
  MatrixXd X(num_features, xvec.size());
  VectorXd Y(xvec.size());
  VectorXd measurements(xvec.size());
  
  for(size_t i = 0; i < xvec.size(); ++i) {
    X.col(i) = xvec[i];
    Y(i) = yvec[i];
    measurements(i) = mvec[i];
  }

  // -- Fit the model.
  PrimeSenseModel model = initial_model_;
  MatrixXd xxt = X * X.transpose();
  ROS_ASSERT(xxt.rows() == initial_model_.numFeatures());
  VectorXd b = X*Y;
  model.weights_ = xxt.ldlt().solve(b);
  cout << "Weights: " << model.weights_.transpose() << endl;
  
  VectorXd pre_differences = measurements - Y;
  double pre_obj = pre_differences.array().pow(2).sum() / (double)Y.rows();
  cout << "Mean error before fitting model: " << pre_obj << endl;

  VectorXd differences = X.transpose() * model.weights_ - Y;
  double obj = differences.array().pow(2).sum() / (double)Y.rows();
  cout << "Mean error after fitting model: " << obj << endl;

  return model;
}

void PixelStats::addPoint(double velo, double asus)
{
  velo_.push_back(velo);
  asus_.push_back(asus);
}

void PixelStats::stats(double* mean, double* stdev, double* num) const
{
  *num = velo_.size();
  
  if(!valid()) {
    *mean = 1;
    *stdev = 0;
    return;
  }
  
  *mean = 0;
  for(size_t i = 0; i < velo_.size(); ++i)
    *mean += velo_[i] / asus_[i];
  *mean /= (double)velo_.size();
  
  *stdev = 0;
  for(size_t i = 0; i < velo_.size(); ++i)
    *stdev += pow(velo_[i] / asus_[i] - *mean, 2);
  *stdev /= (double)velo_.size();
  *stdev = sqrt(*stdev);
}

bool PixelStats::valid() const
{
  return velo_.size() > 5;
}

size_t DepthDistortionLearner::size() const
{
  ROS_ASSERT(frames_.size() == pcds_.size());
  ROS_ASSERT(frames_.size() == transforms_.size());
  return frames_.size();
}
