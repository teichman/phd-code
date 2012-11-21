#include <xpl_calibration/discrete_depth_distortion_model.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

Frustum::Frustum(int smoothing, double bin_depth) :
  max_dist_(15),
  bin_depth_(bin_depth)
{
  num_bins_ = ceil(max_dist_ / bin_depth_);
  counts_ = VectorXf::Ones(num_bins_) * smoothing;
  total_multipliers_ = VectorXf::Ones(num_bins_) * smoothing;
  multipliers_ = VectorXf::Ones(num_bins_);
}

void Frustum::addExample(double ground_truth, double measurement)
{
  scopeLockWrite;
  
  double mult = ground_truth / measurement;
  if(mult > MAX_MULT || mult < MIN_MULT)
    return;
  
  int idx = min(num_bins_ - 1, (int)floor(measurement / bin_depth_));
  ROS_ASSERT(idx >= 0);
  total_multipliers_(idx) += mult;
  ++counts_(idx);
  multipliers_(idx) = total_multipliers_(idx) / counts_(idx);
}

void Frustum::addMultiplier(double measurement, double multiplier)
{
  scopeLockWrite;
  
  ROS_ASSERT(multiplier > 0);
  if(multiplier > MAX_MULT || multiplier < MIN_MULT)
    return;
  
  int idx = min(num_bins_ - 1, (int)floor(measurement / bin_depth_));
  ROS_ASSERT(idx >= 0);
  total_multipliers_(idx) += multiplier;
  ++counts_(idx);
  multipliers_(idx) = total_multipliers_(idx) / counts_(idx);
}

void Frustum::undistort(rgbd::Point* pt) const
{
  double dist = pt->getVector3fMap().norm();
  int idx = min(num_bins_ - 1, (int)floor(dist / bin_depth_));
  ROS_ASSERT(idx >= 0);
  pt->getVector3fMap() *= multipliers_(idx);
}

void Frustum::serialize(std::ostream& out) const
{
  eigen_extensions::serializeScalar(max_dist_, out);
  eigen_extensions::serializeScalar(num_bins_, out);
  eigen_extensions::serializeScalar(bin_depth_, out);
  eigen_extensions::serialize(counts_, out);
  eigen_extensions::serialize(total_multipliers_, out);
  eigen_extensions::serialize(multipliers_, out);
}

void Frustum::deserialize(std::istream& in)
{
  eigen_extensions::deserializeScalar(in, &max_dist_);
  eigen_extensions::deserializeScalar(in, &num_bins_);
  eigen_extensions::deserializeScalar(in, &bin_depth_);
  eigen_extensions::deserialize(in, &counts_);
  eigen_extensions::deserialize(in, &total_multipliers_);
  eigen_extensions::deserialize(in, &multipliers_);
}

DiscreteDepthDistortionModel::DiscreteDepthDistortionModel(const PrimeSenseModel& psm,
							   int bin_width, int bin_height, double bin_depth,
							   int smoothing) :
  psm_(psm),
  bin_width_(bin_width),
  bin_height_(bin_height),
  bin_depth_(bin_depth)
{
  ROS_ASSERT(psm_.width_ % bin_width_ == 0);
  ROS_ASSERT(psm_.height_ % bin_height_ == 0);

  num_bins_x_ = psm_.width_ / bin_width_;
  num_bins_y_ = psm_.height_ / bin_height_;
  frustums_.resize(num_bins_y_);
  for(size_t i = 0; i < frustums_.size(); ++i) {
    frustums_[i].resize(num_bins_x_, NULL);
    for(size_t j = 0; j < frustums_[i].size(); ++j)
      frustums_[i][j] = new Frustum(smoothing, bin_depth);
  }
}

void DiscreteDepthDistortionModel::deleteFrustums()
{
  for(size_t y = 0; y < frustums_.size(); ++y)
    for(size_t x = 0; x < frustums_[y].size(); ++x)
      if(frustums_[y][x])
	delete frustums_[y][x];
}

DiscreteDepthDistortionModel::~DiscreteDepthDistortionModel()
{
  deleteFrustums();
}

void DiscreteDepthDistortionModel::undistort(Frame* frame) const
{
  ROS_ASSERT(psm_.width_ == frame->depth_->cols());
  ROS_ASSERT(psm_.height_ == frame->depth_->rows());

  ProjectivePoint ppt;
  Point pt;
  for(int v = 0; v < psm_.height_; ++v) {
    for(int u = 0; u < psm_.width_; ++u) {
      if(frame->depth_->coeffRef(v, u) == 0)
	continue;

      ppt.v_ = v;
      ppt.u_ = u;
      ppt.z_ = frame->depth_->coeffRef(v, u);
      psm_.project(ppt, &pt);
      frustum(v, u).undistort(&pt);
      psm_.project(pt, &ppt);

      //ROS_ASSERT(ppt.u_ == u && ppt.v_ == v);  // TODO: PrimeSenseModel should project back to exactly the same spot.
      frame->depth_->coeffRef(v, u) = ppt.z_;
    }
  }
}

void DiscreteDepthDistortionModel::accumulate(const rgbd::Frame& measurement, const Eigen::MatrixXd& multipliers)
{
  ROS_ASSERT(psm_.width_ == measurement.depth_->cols());
  ROS_ASSERT(psm_.height_ == measurement.depth_->rows());
  ROS_ASSERT(psm_.width_ == multipliers.cols());
  ROS_ASSERT(psm_.height_ == multipliers.rows());

  ProjectivePoint ppt;
  Point pt;
  for(ppt.v_ = 0; ppt.v_ < psm_.height_; ++ppt.v_) {
    for(ppt.u_ = 0; ppt.u_ < psm_.width_; ++ppt.u_) {
      double mult = multipliers(ppt.v_, ppt.u_);
      if(measurement.depth_->coeffRef(ppt.v_, ppt.u_) == 0) {
	ROS_ASSERT(mult == 0);
	continue;
      }
      if(mult == 0)
	continue;
      
      ppt.z_ = measurement.depth_->coeffRef(ppt.v_, ppt.u_);
      psm_.project(ppt, &pt);
      double meas = pt.getVector3fMap().norm();
      
      frustum(ppt.v_, ppt.u_).addMultiplier(meas, mult);
    }
  }
}

void DiscreteDepthDistortionModel::accumulate(const Frame& ground_truth, const Frame& measurement)
{
  ROS_ASSERT(psm_.width_ == ground_truth.depth_->cols());
  ROS_ASSERT(psm_.height_ == ground_truth.depth_->rows());
  ROS_ASSERT(psm_.width_ == measurement.depth_->cols());
  ROS_ASSERT(psm_.height_ == measurement.depth_->rows());

  ProjectivePoint ppt;
  Point pt;
  for(ppt.v_ = 0; ppt.v_ < psm_.height_; ++ppt.v_) {
    for(ppt.u_ = 0; ppt.u_ < psm_.width_; ++ppt.u_) {
      if(ground_truth.depth_->coeffRef(ppt.v_, ppt.u_) == 0)
	continue;
      if(measurement.depth_->coeffRef(ppt.v_, ppt.u_) == 0)
	continue;
      
      ppt.z_ = ground_truth.depth_->coeffRef(ppt.v_, ppt.u_);
      psm_.project(ppt, &pt);
      double gt = pt.getVector3fMap().norm();

      ppt.z_ = measurement.depth_->coeffRef(ppt.v_, ppt.u_);
      psm_.project(ppt, &pt);
      double meas = pt.getVector3fMap().norm();
      
      frustum(ppt.v_, ppt.u_).addExample(gt, meas);
    }
  }
}

void DiscreteDepthDistortionModel::serialize(std::ostream& out) const
{
  psm_.serialize(out);
  eigen_extensions::serializeScalar(bin_width_, out);
  eigen_extensions::serializeScalar(bin_height_, out);
  eigen_extensions::serializeScalar(bin_depth_, out);
  eigen_extensions::serializeScalar(num_bins_x_, out);
  eigen_extensions::serializeScalar(num_bins_y_, out);

  for(int y = 0; y < num_bins_y_; ++y)
    for(int x = 0; x < num_bins_x_; ++x) 
      frustums_[y][x]->serialize(out);
}

void DiscreteDepthDistortionModel::deserialize(std::istream& in)
{
  psm_.deserialize(in);
  eigen_extensions::deserializeScalar(in, &bin_width_);
  eigen_extensions::deserializeScalar(in, &bin_height_);
  eigen_extensions::deserializeScalar(in, &bin_depth_);
  eigen_extensions::deserializeScalar(in, &num_bins_x_);
  eigen_extensions::deserializeScalar(in, &num_bins_y_);

  deleteFrustums();
  
  frustums_.resize(num_bins_y_);
  for(size_t y = 0; y < frustums_.size(); ++y) {
    frustums_[y].resize(num_bins_x_, NULL);
    for(size_t x = 0; x < frustums_[y].size(); ++x) {
      frustums_[y][x] = new Frustum;
      frustums_[y][x]->deserialize(in);
    }
  }
}

Frustum& DiscreteDepthDistortionModel::frustum(int y, int x)
{
  ROS_ASSERT(x >= 0 && x < psm_.width_);
  ROS_ASSERT(y >= 0 && y < psm_.height_);
  int xidx = x / bin_width_;
  int yidx = y / bin_height_;
  return (*frustums_[yidx][xidx]);
}

const Frustum& DiscreteDepthDistortionModel::frustum(int y, int x) const
{
  ROS_ASSERT(x >= 0 && x < psm_.width_);
  ROS_ASSERT(y >= 0 && y < psm_.height_);
  int xidx = x / bin_width_;
  int yidx = y / bin_height_;
  return (*frustums_[yidx][xidx]);
}

