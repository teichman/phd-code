#include <xpl_calibration/discrete_depth_distortion_model.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
namespace bfs = boost::filesystem;

Frustum::Frustum(int smoothing, double bin_depth) :
  max_dist_(10),
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


inline int Frustum::index(double z) const
{
  return min(num_bins_ - 1, (int)floor(z / bin_depth_));
}
  
inline void Frustum::undistort(double* z) const
{
  *z *= multipliers_.coeffRef(index(*z));
}

// inline void Frustum::undistort(int idx, float* z, float* mult) const
// {
//   *mult = multipliers_.coeffRef(idx);
//   *z *= *mult;
// }

void Frustum::interpolatedUndistort(double* z) const
{
  int idx = index(*z);
  double start = bin_depth_ * idx;
  int idx1;
  if(*z - start < bin_depth_ / 2)
    idx1 = idx;
  else
    idx1 = idx + 1;
  int idx0 = idx1 - 1;
  if(idx0 < 0 || idx1 >= num_bins_ || counts_(idx0) < 50 || counts_(idx1) < 50) {
    undistort(z);
    return;
  }

  double z0 = (idx0 + 1) * bin_depth_ - bin_depth_ * 0.5;
  // ROS_ASSERT(z0 <= *z && z1 >= *z);
  // if(!(fabs(z1 - z0 - bin_depth_) < 1e-6)) {
  //   cout << z0 << " " << z1 << " " << bin_depth_ << endl;
  //   ROS_ASSERT(0);
  // }

  double coeff1 = (*z - z0) / bin_depth_;
  //ROS_ASSERT(coeff1 >= 0 && coeff1 <= 1);
  double coeff0 = 1.0 - coeff1;
  double mult = coeff0 * multipliers_.coeffRef(idx0) + coeff1 * multipliers_.coeffRef(idx1);
  *z *= mult;
}

// inline void Frustum::interpolatedUndistort(int idx, float* z, float* mult) const
// {
// }

// void Frustum::undistort(rgbd::Point* pt) const
// {
//   double dist = pt->getVector3fMap().norm();
//   int idx = min(num_bins_ - 1, (int)floor(dist / bin_depth_));
//   ROS_ASSERT(idx >= 0);
//   pt->getVector3fMap() *= multipliers_(idx);
// }

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

DiscreteDepthDistortionModel::DiscreteDepthDistortionModel(const DiscreteDepthDistortionModel& other)
{
  *this = other;
}

DiscreteDepthDistortionModel& DiscreteDepthDistortionModel::operator=(const DiscreteDepthDistortionModel& other)
{
  psm_ = other.psm_;
  width_ = other.width_;
  height_ = other.height_;
  bin_width_ = other.bin_width_;
  bin_height_ = other.bin_height_;
  bin_depth_ = other.bin_depth_;
  num_bins_x_ = other.num_bins_x_;
  num_bins_y_ = other.num_bins_y_;
  idx_cache_ = other.idx_cache_;
  multiplier_cache_ = other.multiplier_cache_;
  
  frustums_ = other.frustums_;
  for(size_t i = 0; i < frustums_.size(); ++i)
    for(size_t j = 0; j < frustums_[i].size(); ++j)
      frustums_[i][j] = new Frustum(*other.frustums_[i][j]);

  return *this;
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
  idx_cache_ = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Ones(psm_.height_, psm_.width_) * -1;
  multiplier_cache_ = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(psm_.height_, psm_.width_);
  //multiplier_cache_ = MatrixXf::Zero(num_bins_y_, num_bins_x_);  // Why does this compile?
  
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
  #pragma omp parallel for
  for(int v = 0; v < psm_.height_; ++v) {
    for(int u = 0; u < psm_.width_; ++u) {
      if(frame->depth_->coeffRef(v, u) == 0)
	continue;

      // Non-caching version.
      double z = frame->depth_->coeffRef(v, u) * 0.001;
      //frustum(v, u).undistort(&z);
      frustum(v, u).interpolatedUndistort(&z);
      frame->depth_->coeffRef(v, u) = z * 1000;

      // Caching version.
      // float z = frame->depth_->coeffRef(v, u) * 0.001;
      // int idx = frustum(v, u).index(z);
      // if(idx_cache_.coeffRef(v, u) == idx)
      // 	z *= multiplier_cache_.coeffRef(v, u);
      // else {
      // 	float mult;
      // 	frustum(v, u).undistort(idx, &z, &mult);
      // 	idx_cache_.coeffRef(v, u) = idx;
      // 	multiplier_cache_.coeffRef(v, u) = mult;
      // }
      // frame->depth_->coeffRef(v, u) = z * 1000;
    }
  }
}

void DiscreteDepthDistortionModel::addExample(const ProjectivePoint& ppt, double ground_truth, double measurement)
{
  frustum(ppt.v_, ppt.u_).addExample(ground_truth, measurement);
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
      
      // ppt.z_ = measurement.depth_->coeffRef(ppt.v_, ppt.u_);
      // psm_.project(ppt, &pt);
      // double meas = pt.getVector3fMap().norm();
      // frustum(ppt.v_, ppt.u_).addMultiplier(meas, mult);

      double z = measurement.depth_->coeffRef(ppt.v_, ppt.u_) * 0.001;
      frustum(ppt.v_, ppt.u_).addMultiplier(z, mult);
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
      
      // ppt.z_ = ground_truth.depth_->coeffRef(ppt.v_, ppt.u_);
      // psm_.project(ppt, &pt);
      // double gt = pt.getVector3fMap().norm();
      // ppt.z_ = measurement.depth_->coeffRef(ppt.v_, ppt.u_);
      // psm_.project(ppt, &pt);
      // double meas = pt.getVector3fMap().norm();
      // frustum(ppt.v_, ppt.u_).addExample(gt, meas);

      double gt = ground_truth.depth_->coeffRef(ppt.v_, ppt.u_) * 0.001;
      double meas = measurement.depth_->coeffRef(ppt.v_, ppt.u_) * 0.001;
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

  idx_cache_ = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Ones(psm_.height_, psm_.width_) * -1;
  multiplier_cache_ = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(psm_.height_, psm_.width_);
}

void DiscreteDepthDistortionModel::visualize(const std::string& dir) const
{
  if(!bfs::exists(dir))
    bfs::create_directory(dir);
  else
    ROS_ASSERT(bfs::is_directory(dir));
  
  const Frustum& reference_frustum = *frustums_[0][0];
  int num_layers = reference_frustum.num_bins_;

  // -- Set up for combined imagery.
  int horiz_divider = 10;
  int vert_divider = 20;
  cv::Mat3b mega(cv::Size(psm_.width_ * 2 + horiz_divider, psm_.height_ * num_layers + vert_divider * (num_layers + 2)), cv::Vec3b(0, 0, 0));
  vector<int> pub_layers;
  pub_layers.push_back(1);
  pub_layers.push_back(2);
  pub_layers.push_back(3);
  cv::Mat3b pub(cv::Size(psm_.width_, psm_.height_ * pub_layers.size() + vert_divider * (pub_layers.size() + 2)), cv::Vec3b(255, 255, 255));
  
  for(int i = 0; i < num_layers; ++i) {
    // -- Determine the path to save the image for this layer.
    char buffer[50];
    float mindepth = reference_frustum.bin_depth_ * i;
    float maxdepth = reference_frustum.bin_depth_ * (i + 1);
    sprintf(buffer, "%05.2f-%05.2f", mindepth, maxdepth);
    ostringstream oss;
    oss << dir << "/multipliers_" << buffer << ".png";

    // -- Compute the multipliers visualization for this layer.
    //    Multiplier of 1 is black, >1 is red, <1 is blue.  Think redshift.
    cv::Mat3b mult(cv::Size(psm_.width_, psm_.height_), cv::Vec3b(0, 0, 0));
    for(int y = 0; y < mult.rows; ++y) {
      for(int x = 0; x < mult.cols; ++x) {
	const Frustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
	float val = frustum.multipliers_(i);
	if(val > 1)
	  mult(y, x)[2] = min(255., 255 * (val - 1.0) / 0.25);
	if(val < 1)
	  mult(y, x)[0] = min(255., 255 * (1.0 - val) / 0.25);
      }
    }
    cv::imwrite(oss.str(), mult);

    // -- Compute the counts visualization for this layer.
    //    0 is black, 100 is white.
    cv::Mat3b count(cv::Size(psm_.width_, psm_.height_), cv::Vec3b(0, 0, 0));
    for(int y = 0; y < count.rows; ++y) {
      for(int x = 0; x < count.cols; ++x) {
	const Frustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
	uchar val = min(255., (double)(255 * frustum.counts_(i) / 100));
	count(y, x)[0] = val;
	count(y, x)[1] = val;
	count(y, x)[2] = val;
      }
    }
    oss.str("");
    oss << dir << "/counts_" << buffer << ".png";
    cv::imwrite(oss.str(), count);

    // -- Make images showing the two, side-by-side.
    cv::Mat3b combined(cv::Size(psm_.width_ * 2 + horiz_divider, psm_.height_), cv::Vec3b(0, 0, 0));
    for(int y = 0; y < combined.rows; ++y) {
      for(int x = 0; x < combined.cols; ++x) {
	if(x < count.cols)
	  combined(y, x) = count(y, x);
	else if(x > count.cols + horiz_divider)
	  combined(y, x) = mult(y, x - count.cols - horiz_divider);
      }
    }
    oss.str("");
    oss << dir << "/combined_" << buffer << ".png";
    cv::imwrite(oss.str(), combined);

    // -- Append to the mega image.
    for(int y = 0; y < combined.rows; ++y)
      for(int x = 0; x < combined.cols; ++x)
	mega(y + i * (combined.rows + vert_divider) + vert_divider, x) = combined(y, x);

    // -- Compute the publication multipliers visualization for this layer.
    //    Multiplier of 1 is white, >1 is red, <1 is blue.  Think redshift.
    cv::Mat3b pubmult(cv::Size(psm_.width_, psm_.height_), cv::Vec3b(255, 255, 255));
    for(int y = 0; y < pubmult.rows; ++y) {
      for(int x = 0; x < pubmult.cols; ++x) {
	const Frustum& frustum = *frustums_[y / bin_height_][x / bin_width_];
	float val = frustum.multipliers_(i);
	if(val > 1) {
	  pubmult(y, x)[0] = 255 - min(255., 255 * (val - 1.0) / 0.1);
	  pubmult(y, x)[1] = 255 - min(255., 255 * (val - 1.0) / 0.1);
	}
	if(val < 1) {
	  pubmult(y, x)[1] = 255 - min(255., 255 * (1.0 - val) / 0.1);
	  pubmult(y, x)[2] = 255 - min(255., 255 * (1.0 - val) / 0.1);
	}
      }
    }
  
    // -- Append to publication image.
    for(size_t j = 0; j < pub_layers.size(); ++j)
      if(pub_layers[j] == i)
	for(int y = 0; y < pubmult.rows; ++y)
	  for(int x = 0; x < pubmult.cols; ++x)
	    pub(y + j * (pubmult.rows + vert_divider) + vert_divider, x) = pubmult(y, x);
  }
  
  // -- Add a white bar at the top and bottom for reference.
  for(int y = 0; y < mega.rows; ++y)
    if(y < vert_divider || y > mega.rows - vert_divider)
      for(int x = 0; x < mega.cols; ++x)
	mega(y, x) = cv::Vec3b(255, 255, 255);
  
  // -- Save mega image.
  ostringstream oss;
  oss << dir << "/mega.png";
  cv::imwrite(oss.str(), mega);

  // -- Save a small version for easy loading.
  cv::Mat3b mega_scaled;
  cv::resize(mega, mega_scaled, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
  oss.str("");
  oss << dir << "/mega_scaled.png";
  cv::imwrite(oss.str(), mega_scaled);

  // -- Save publication image.
  oss.str("");
  oss << dir << "/pub";
  for(size_t i = 0; i < pub_layers.size(); ++i)
    oss << "-" << setw(2) << setfill('0') << pub_layers[i];
  oss << ".png";
  cv::imwrite(oss.str(), pub);
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

