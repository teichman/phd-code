#include <sentinel/background_model.h>
#include <ros/assert.h>
#include <timer/timer.h>

using namespace std;
using namespace Eigen;

DepthHistogram::DepthHistogram(double min_depth, double max_depth, double binwidth,
                               int x, int y) :
  debug_(false),
  x_(x),
  y_(y)
{
  initialize(min_depth, max_depth, binwidth);
}

void DepthHistogram::initialize(double min_depth, double max_depth, double binwidth)
{
  clear();

  total_ = 0;
  min_depth_ = min_depth;
  max_depth_ = max_depth;
  binwidth_ = binwidth;
  inv_binwidth_ = 1.0 / binwidth_;

  int num_bins = ceil((max_depth_ - min_depth_) / binwidth_);
  bins_.resize(num_bins, 0);
  lower_limits_.resize(num_bins);
  for(int i = 0; i < num_bins; ++i)
    lower_limits_[i] = min_depth_ + i * binwidth_;
}

void DepthHistogram::increment(double z, int num)
{
  ROS_ASSERT(min_depth_ < z && z < max_depth_);
  size_t lower_idx;
  double upper_weight;
  indices(z, &lower_idx, &upper_weight);
  
  total_ += num;
  bins_[lower_idx] += num * (1.0 - upper_weight);
  bins_[lower_idx+1] += num * upper_weight;

  if(debug_) {
    cout << "#################### DepthHistogram::increment" << endl;
    cout << "Incremented by " << num << " at depth " << z << endl;
    cout << status() << endl;
    cout << "#################### aoeuaoeuaoeu" << endl;
  }
}

std::string DepthHistogram::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "x: " << x_ << endl;
  oss << prefix << "y: " << y_ << endl;
  oss << prefix << "min_depth: " << min_depth_ << endl;
  oss << prefix << "max_depth_: " << max_depth_ << endl;
  oss << prefix << "binwidth_: " << binwidth_ << endl;
  oss << prefix << "inv_binwidth_: " << inv_binwidth_ << endl;
  oss << prefix << "total_: " << total_ << endl;
  oss << prefix << "Bins: " << endl;
  for(size_t i = 0; i < lower_limits_.size(); ++i)
    oss << prefix << "  " << lower_limits_[i] << ": " << bins_[i] << endl;

  return oss.str();
}
  

void DepthHistogram::clear()
{
  min_depth_ = -1;
  max_depth_ = -1;
  binwidth_ = -1;
  inv_binwidth_ = -1;
  lower_limits_.clear();
  bins_.clear();
  total_ = 0;
}

BackgroundModel::BackgroundModel(int width, int height,
                                 int width_step, int height_step,
                                 double min_pct,
                                 double min_depth, double max_depth,
                                 double bin_width) :
  width_(width),
  height_(height),
  width_step_(width_step),
  height_step_(height_step),
  min_pct_(min_pct),
  min_depth_(min_depth),
  max_depth_(max_depth),
  bin_width_(bin_width)
{
  // -- Set up the space transform.
  // f(x) = ax^2 + bx + c
  // f'(x) = 2ax + b
  // Constraints:
  // f(0.5) = 0.5
  // f(5) = 5
  // mult * f'(5) = f'(0.5)
  double mult = 5;
  MatrixXd A(3, 3);
  A << 0.25, 0.5, 1,
    25, 5, 1,
    mult * 10 - 1, mult - 1, 0;
  VectorXd b(3);
  b << 0.5, 5, 0;
  weights_ = A.colPivHouseholderQr().solve(b);

  ROS_ASSERT(fabs(0.5 - transform(0.5)) < 1e-6);
  ROS_ASSERT(fabs(5 - transform(5)) < 1e-6);
  ROS_ASSERT(fabs(mult*transformDerivative(5) - transformDerivative(0.5)) < 1e-6);

  ROS_ASSERT(height_step_ == 1 || height_step_ % 2 == 0);
  ROS_ASSERT(width_step_ == 1 || width_step_ % 2 == 0);
  ROS_ASSERT(height_ % height_step_ == 0 && width_ % width_step_ == 0);

  blocks_per_row_ = width_ / width_step_;
  blocks_per_col_ = height_ / height_step_;
  block_img_ = cv::Mat1b(cv::Size(blocks_per_col_, blocks_per_row_), 0);
  dilated_block_img_ = cv::Mat1b(cv::Size(blocks_per_col_, blocks_per_row_), 0);
  
  size_t num = 0;
  for(int y = height_step_ / 2; y < height; y += height_step_)
    for(int x = width_step_ / 2; x < width; x += width_step_)
      ++num;
  histograms_.reserve(num);
  for(int y = height_step_ / 2; y < height; y += height_step_)
    for(int x = width_step_ / 2; x < width; x += width_step_)
      histograms_.push_back(DepthHistogram(min_depth_, max_depth_, bin_width_, x, y));

  cout << "Initialized " << histograms_.size() << " histograms." << endl;

  // -- Print out bin widths.
  const DepthHistogram& hist = histograms_[0];
  for(size_t i = 0; i < hist.lower_limits_.size(); ++i)
    cout << "Bin " << i << ": " << inverseTransform(hist.lower_limits_[i]) << endl;
}

void BackgroundModel::increment(openni::VideoFrameRef depth, int num)
{
  #if JARVIS_DEBUG
  ScopedTimer st("BackgroundModel::increment");
  #endif

  ROS_ASSERT(depth.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM);
  uint16_t* data = (uint16_t*)depth.getData();
  size_t idx = 0;
  for(int y = height_step_ / 2; y < height_; y += height_step_) {
    for(int x = width_step_ / 2; x < width_; x += width_step_, ++idx) {
      ROS_ASSERT(depth.getWidth() > 0 && depth.getHeight() > 0);
      ROS_ASSERT(depth.getDataSize() > 0);
      ROS_ASSERT(y * depth.getWidth() + x < depth.getDataSize());
      double z = data[y * depth.getWidth() + x] * 0.001;
      if(MIN_DEPTH < z && z < MAX_DEPTH)
        histograms_[idx].increment(transform(z), num);
    }
  }
}

void BackgroundModel::predict(openni::VideoFrameRef depth,
                              vector<uint32_t>* indices,
                              vector<uint32_t>* fg_markers,
                              vector<uint32_t>* bg_fringe_markers)
{
  ROS_ASSERT(width_ == depth.getWidth());

  indices->clear();
  fg_markers->clear();
  bg_fringe_markers->clear();
  
  size_t idx = 0;
  uint16_t* data = (uint16_t*)depth.getData();

  // -- Fill in foreground.
  block_img_ = 0;
  for(int y = height_step_ / 2; y < height_; y += height_step_) {
    for(int x = width_step_ / 2; x < width_; x += width_step_, ++idx) {
      uint16_t val = data[y * width_ + x];
      if(val == 0)
        continue;

      double z = val * 0.001;
      if(MIN_DEPTH > z || z > MAX_DEPTH)
        continue;
      
      double pct = histograms_[idx].getNum(transform(z)) / histograms_[idx].total();
      if(pct < min_pct_) {
        fg_markers->push_back(y * width_ + x);
        int r = idx / blocks_per_row_;
        int c = idx - r * blocks_per_row_;
        block_img_(r, c) = 255;
        for(int y2 = r * height_step_; y2 < (r+1) * height_step_; ++y2)
          for(int x2 = c * width_step_; x2 < (c+1) * width_step_; ++x2)
            indices->push_back(y2 * width_ + x2);
      }
    }
  }

  // -- Fill in background fringe with 127s.
  idx = 0;
  cv::dilate(block_img_, dilated_block_img_, cv::Mat(), cv::Point(-1, -1), 1);
  for(int r = 0; r < block_img_.rows; ++r) {
    for(int c = 0; c < block_img_.cols; ++c) {
      if(block_img_(r, c) == 0 && dilated_block_img_(r, c) == 255) {
        int y = r * height_step_ + height_step_ / 2;
        int x = c * width_step_ + width_step_ / 2;
        bg_fringe_markers->push_back(y * width_ + x);
        for(int y2 = r * height_step_; y2 < (r+1) * height_step_; ++y2)
          for(int x2 = c * width_step_; x2 < (c+1) * width_step_; ++x2)
            indices->push_back(y2 * width_ + x2);
      }
    }
  }
}

double BackgroundModel::transform(double x) const
{
  return weights_.coeffRef(0) * x * x + weights_.coeffRef(1) * x + weights_.coeffRef(2);
}

double BackgroundModel::transformDerivative(double x) const
{
  return 2 * weights_.coeffRef(0) * x + weights_.coeffRef(1);
}

double BackgroundModel::inverseTransform(double x) const
{
  double a = weights_(0);
  double b = weights_(1);
  double c = weights_(2) - x;

  ROS_ASSERT(b*b - 4*a*c >= 0);
  double val = sqrt(b*b - 4*a*c);
  double result0 = (-b + val) / (2*a);
  double result1 = (-b - val) / (2*a);

  if(min_depth_ - 1e-6 < result0 && result0 < max_depth_ + 1e-6) {
    ROS_ASSERT(!(min_depth_ - 1e-6 < result1 && result1 < max_depth_ + 1e-6));
    return result0;
  }
  else {
    ROS_ASSERT(min_depth_ - 1e-6 < result1 && result1 < max_depth_ + 1e-6);
    ROS_ASSERT(!(min_depth_ - 1e-6 < result0 && result0 < max_depth_ + 1e-6));
    return result1;
  }
}

void BackgroundModel::debug(int x, int y)
{
  for(size_t i = 0; i < histograms_.size(); ++i)
    histograms_[i].debug_ = false;
  
  ROS_ASSERT(width_step_ == 1 && height_step_ == 1);
  int idx = y * width_ + x;
  histograms_[idx].debug_ = true;
}
