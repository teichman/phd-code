#include <sentinel/background_model.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

DepthHistogram::DepthHistogram(double min_depth, double max_depth, double binwidth) :
  finalized_(false)
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

  int num_bins = ceil((max_depth_ - min_depth_) / binwidth_);
  bins_.resize(num_bins, 0);
  lower_limits_.resize(num_bins);
  for(int i = 0; i < num_bins; ++i)
    lower_limits_[i] = min_depth_ + i * binwidth_;
}

void DepthHistogram::increment(double z, int num)
{
  int idx = (z - min_depth_) / binwidth_;
  if(idx < 0 || idx > (int)bins_.size())
    return;

  total_ += num;
  bins_[idx] += num;
  finalized_ = false;
}

void DepthHistogram::clear()
{
  min_depth_ = -1;
  max_depth_ = -1;
  binwidth_ = -1;
  lower_limits_.clear();
  bins_.clear();
  num_nearby_.clear();
  total_ = 0;
  finalized_ = false;
}

void DepthHistogram::finalize()
{
  num_nearby_.resize(bins_.size());
  for(int idx = 0; idx < (int)num_nearby_.size(); ++idx)
    for(int i = idx - 1; i <= idx + 1; ++i)
      if(i >= 0 && i < (int)bins_.size())
        num_nearby_[idx] = max(bins_[i], num_nearby_[idx]);

  finalized_ = true;
}

int DepthHistogram::getNumNearby(double z) const
{
  ROS_ASSERT(num_nearby_.size() == bins_.size());
  ROS_ASSERT(finalized_);
  
  int idx = (z - min_depth_) / binwidth_;
  if(idx < 0 || idx >= (int)bins_.size())
    return -1;
  else
    return num_nearby_[idx];
}

BackgroundModel::BackgroundModel(int num_pixels, double min_pct,
                                 double max_depth, double res) :
  min_pct_(min_pct),
  max_depth_(max_depth),
  res_(res),
  finalized_(false)
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
  ROS_ASSERT(depth.rows() * depth.cols() == (int)histograms_.size());
  // -- Compute histograms of z values.
  int idx = 0;
  for(int y = 0; y < depth.rows(); ++y)
    for(int x = 0; x < depth.cols(); ++x, ++idx)
      if(depth(y, x) != 0)
        histograms_[idx]->increment(depth(y, x) / 1000.0, num);

  finalized_ = false;
}

void BackgroundModel::finalize()
{
  for(size_t i = 0; i < histograms_.size(); ++i)
    histograms_[i]->finalize();

  finalized_ = true;
}

bool BackgroundModel::isBackground(size_t idx, double z) const
{
  ROS_ASSERT(idx < histograms_.size());
  ROS_ASSERT(finalized_);

  int num = histograms_[idx]->getNumNearby(z);
  if(num == -1)
    return true;
    
  double pct = (double)num / (double)histograms_[idx]->total();
  if(pct > min_pct_)
    return true;
  else
    return false;
}
