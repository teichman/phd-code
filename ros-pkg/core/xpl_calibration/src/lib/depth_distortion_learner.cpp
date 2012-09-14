#include <xpl_calibration/depth_distortion_learner.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

DepthDistortionLearner::DepthDistortionLearner(const PrimeSenseModel& initial_model) :
  initial_model_(initial_model),
  coverage_map_(0.1, initial_model.height_, initial_model.width_)
{
}

void DepthDistortionLearner::addFrame(Frame frame,
				      rgbd::Cloud::ConstPtr pcd)
{
  coverage_map_.addFrame(frame);
  frames_.push_back(frame);
  pcds_.push_back(pcd);
}

PrimeSenseModel DepthDistortionLearner::fitFocalLength()
{
  FocalLengthMDE::Ptr objective(new FocalLengthMDE(initial_model_, frames_, pcds_));
  GridSearch gs(1);
  gs.verbose_ = false;
  gs.objective_ = objective;
  gs.num_scalings_ = 6;
  gs.max_resolutions_ << 10;
  gs.grid_radii_ << 5;
  gs.scale_factors_ << 0.5;


  ArrayXd init(1);
  init(0) = initial_model_.fx_;

  cout << "Initial f: " << init(0) << endl;
  cout << "Initial objective: " << objective->eval(init) << endl;
  ArrayXd x = gs.search(init);
  cout << "GridSearch solution: " << x.transpose() << endl;
  cout << "Final objective: " << objective->eval(x) << endl;

  PrimeSenseModel model = initial_model_;
  model.fx_ = x(0);
  model.fy_ = x(0);
  return model;
}

PrimeSenseModel DepthDistortionLearner::fitModel()
{
  ROS_ASSERT(frames_.size() == pcds_.size());
  // -- Make sure the initial model is reasonable.
  //    It's going to be used for projection assuming no depth distortion.
  if(initial_model_.hasDepthDistortionModel()) {
    ROS_FATAL("DepthDistortionLearner should not have an initial_model_ with a depth distortion model already built in to it.");
    ROS_FATAL_STREAM(initial_model_.weights_);
    abort();
  }

  double min_mult = 0.8;
  double max_mult = 1.2;
  vector<VectorXd> xvec;
  vector<double> yvec;
  vector<double> mvec;
  xvec.reserve(1e6);
  yvec.reserve(1e6);
  mvec.reserve(1e6);
  
  for(size_t i = 0; i < frames_.size(); ++i) {
    Frame mapframe;
    initial_model_.cloudToFrame(*pcds_[i], &mapframe);
    const DepthMat& depth = *frames_[i].depth_;
    const DepthMat& mapdepth = *mapframe.depth_;

    ProjectivePoint ppt;
    Point pt;
    for(ppt.v_ = 0; ppt.v_ < depth.rows(); ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < depth.cols(); ++ppt.u_) {
	if(mapdepth(ppt.v_, ppt.u_) == 0 || depth(ppt.v_, ppt.u_) == 0)
	  continue;

	// if(rand() % 5 != 0)
	//   continue;
	
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
  model.weights_ = regressRegularized(X, Y); 

  return model;
}

Eigen::VectorXd DepthDistortionLearner::regress(const Eigen::MatrixXd& X, const Eigen::VectorXd& Y) const
{
  MatrixXd xxt = X * X.transpose();
  VectorXd b = X*Y;
  return xxt.ldlt().solve(b);
}

Eigen::VectorXd DepthDistortionLearner::regressRegularized(const Eigen::MatrixXd& X, const Eigen::VectorXd& Y) const
{
  double gamma = 0.1;
  MatrixXd xxtr = X * X.transpose() + MatrixXd::Identity(X.rows(), X.rows()) * gamma;
  VectorXd b = X*Y;
  return xxtr.ldlt().solve(b);
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
  return frames_.size();
}

CoverageMap::CoverageMap(double scale, int rows, int cols) :
  min_dist_(0.8),
  max_dist_(10),
  scale_(scale),
  rows_(rows),
  cols_(cols)
{
  bins_.resize(rows * scale);
  for(size_t y = 0; y < bins_.size(); ++y) {
    bins_[y].resize(cols * scale);
    for(size_t x = 0; x < bins_[y].size(); ++x) {
      bins_[y][x].reserve(100);
    }
  }
}

void CoverageMap::addFrame(rgbd::Frame frame)
{
  const DepthMat& depth = *frame.depth_;
  for(int y = 0; y < depth.rows(); ++y) {
    for(int x = 0; x < depth.cols(); ++x) {
      int v = ((double)y / depth.rows()) * rows();
      int u = ((double)x / depth.cols()) * cols();
      if(u < 0)
	u = 0;
      if(u >= (int)cols())
	u = cols();
      if(v < 0)
	v = 0;
      if(v >= (int)rows())
	v = rows();
      bins_[v][u].push_back(depth(y, x) * 0.001);
    }
  }
}

cv::Mat3b CoverageMap::computeImage() const
{
  cv::Mat3b small(rows(), cols());
  for(size_t y = 0; y < rows(); ++y) {
    for(size_t x = 0; x < cols(); ++x) {
      small(y, x) = colorizeBin(bins_[y][x]);
    }
  }

  cv::Mat3b img;
  cv::resize(small, img, cv::Size(cols_, rows_), cv::INTER_NEAREST);
  return img;
}

cv::Vec3b CoverageMap::colorizeBin(const vector<double>& bin) const
{
  int num_bins = 10;

  VectorXd counts = VectorXd::Zero(num_bins);
  double range = max_dist_ - min_dist_;
  double width = range / num_bins;
  for(size_t i = 0; i < bin.size(); ++i) {
    int idx = floor((bin[i] - min_dist_) / width);
    if(idx < 0)
      idx = 0;
    if(idx >= num_bins)
      idx = num_bins;
    ++counts(idx);
  }

  int filled_criterion = 100;
  int num_filled = 0;
  for(int i = 0; i < counts.rows(); ++i)
    if(counts(i) > filled_criterion)
      ++num_filled;

  double frac_filled = (double)num_filled / num_bins;
  return cv::Vec3b(0, 255 * frac_filled, 255 * (1.0 - frac_filled));
}

void CoverageMap::clear()
{
  for(size_t y = 0; y < rows(); ++y) 
    for(size_t x = 0; x < cols(); ++x)
      bins_[y][x].clear();
}

