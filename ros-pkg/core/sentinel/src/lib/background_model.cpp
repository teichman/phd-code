#include <sentinel/background_model.h>
#include <ros/assert.h>
#include <timer/timer.h>

using namespace std;
using namespace Eigen;

DepthHistogram::DepthHistogram(double min_depth, double max_depth, double binwidth) 
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
  size_t lower_idx;
  double upper_weight;
  indices(z, &lower_idx, &upper_weight);
  
  total_ += num;
  bins_[lower_idx] += num * (1.0 - upper_weight);
  bins_[lower_idx+1] += num * upper_weight;
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

void DepthHistogram::indices(double z, size_t* lower_idx, double* upper_weight) const
{
  *lower_idx = max<size_t>(0, (z - min_depth_) * inv_binwidth_);
  *upper_weight = (z - lower_limits_[*lower_idx]) * inv_binwidth_;
}

double DepthHistogram::getNum(double z) const
{
  size_t lower_idx;
  double upper_weight;
  indices(z, &lower_idx, &upper_weight);

  return bins_[lower_idx] * (1.0 - upper_weight) + bins_[lower_idx + 1] * (upper_weight);
}

BackgroundModel::BackgroundModel(int num_pixels, double min_pct,
                                 double max_depth, double res) :
  min_pct_(min_pct),
  max_depth_(max_depth),
  res_(res)
{
  cout << "Initializing with " << num_pixels << " pixels." << endl;
  histograms_.reserve(num_pixels);
  for(int i = 0; i < num_pixels; ++i) { 
    DepthHistogram::Ptr dh(new DepthHistogram(0, max_depth_, res_));
    histograms_.push_back(dh);
  }
}

void BackgroundModel::increment(const DepthMat& depth, int num)
{
  #if TIMING
  ScopedTimer st("BackgroundModel::increment");
  #endif
  
  ROS_ASSERT(depth.rows() * depth.cols() == (int)histograms_.size());

  // -- Compute histograms of z values.
  int idx = 0;
  for(int y = 0; y < depth.rows(); ++y)
    for(int x = 0; x < depth.cols(); ++x, ++idx)
      if(depth(y, x) != 0)
        histograms_[idx]->increment(depth(y, x) / 1000.0, num);
}

bool BackgroundModel::isBackground(size_t idx, double z) const
{
  ROS_ASSERT(idx < histograms_.size());
  return histograms_[idx]->getNum(z) / histograms_[idx]->total() > min_pct_;
}
