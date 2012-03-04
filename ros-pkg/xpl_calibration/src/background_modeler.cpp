#include <xpl_calibration/background_modeler.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

DepthHistogram::DepthHistogram(double minval, double maxval, double binwidth)
{
  initialize(minval, maxval, binwidth);
}

void DepthHistogram::initialize(double minval, double maxval, double binwidth)
{
  clear();
  
  minval_ = minval;
  maxval_ = maxval;
  binwidth_ = binwidth;

  int num_bins = ceil((maxval_ - minval_) / binwidth_);
  bins_.resize(num_bins, 0);
  lower_limits_.resize(num_bins);
  for(int i = 0; i < num_bins; ++i)
    lower_limits_[i] = minval_ + i * binwidth_;
}

void DepthHistogram::insert(double val)
{
  int idx = (val - minval_) / binwidth_;
  if(idx < 0 || idx > (int)bins_.size())
    return;

  ++bins_[idx];
}

void DepthHistogram::clear()
{
  minval_ = -1;
  maxval_ = -1;
  binwidth_ = -1;
  lower_limits_.clear();
  bins_.clear();
}

void DepthHistogram::getBackground(int minpts, double* minval, double* maxval) const
{
  // -- Get furthest bin with at least minpts.
  int idx = -1;
  for(size_t i = 0; i < bins_.size(); ++i)
    if(bins_[i] > minpts)
      idx = i;

  if(idx == -1) {
    *minval = -1;
    *maxval = -1;
  }
  else { 
    *minval = lower_limits_[idx];
    *maxval = lower_limits_[idx] + binwidth_;
  }
}

void BackgroundModeler::compute()
{
  // -- Initialize.
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  ROS_ASSERT(seq.pcds_[0]->isOrganized());
  size_t num_pixels = seq.pcds_[0]->size();
  
  if(histograms_.size() != num_pixels) {
    histograms_.clear();
    histograms_.reserve(num_pixels);
    for(size_t i = 0; i < num_pixels; ++i) { 
      DepthHistogram::Ptr dh(new DepthHistogram(0, param<double>("MaxDepth"), param<double>("Resolution")));
      histograms_.push_back(dh);
    }
  }
  else {
    for(size_t i = 0; i < num_pixels; ++i)
      histograms_[i]->clear();
  }
  
  // -- Compute histograms of z values.
  HighResTimer hrt("Accumulating");
  hrt.start();
  for(size_t i = 0; i < seq.size(); ++i) {
    const Cloud& pcd = *seq.pcds_[i];
    for(size_t j = 0; j < pcd.size(); ++j) {
      if(!isinf(pcd[j].z))
	histograms_[j]->insert(pcd[j].z);
    }
  }
  hrt.stop();
  cout << hrt.report() << endl;

  // -- Get the background bin.
  hrt.reset("Finding max dist");
  hrt.start();
  min_distances_.clear();
  max_distances_.clear();
  for(size_t i = 0; i < histograms_.size(); ++i) {
    double min;
    double max;
    histograms_[i]->getBackground(param<int>("MinNumPoints"), &min, &max);
    min_distances_.push_back(min);
    max_distances_.push_back(max);
  }
  hrt.stop();
  cout << hrt.report() << endl;
  
  push<const vector<double>*>("MinDistances", &min_distances_);
  push<const vector<double>*>("MaxDistances", &max_distances_);
}

void BackgroundModeler::debug() const
{
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  const Cloud& pcd = *seq.pcds_[0];
  cv::Mat1f bg(cv::Size(pcd.width, pcd.height), 0);

  double max = -numeric_limits<double>::max();
  for(int y = 0; y < bg.rows; ++y) {
    for(int x = 0; x < bg.cols; ++x) {
      int idx = y * bg.cols + x;
      bg(y, x) = max_distances_[idx];
      if(max_distances_[idx] > max)
	max = max_distances_[idx];
    }
  }

  cout << "Max: " << max << endl;
  for(int y = 0; y < bg.rows; ++y)
    for(int x = 0; x < bg.cols; ++x)
      bg(y, x) /= max;

  cv::Mat1b vis(bg.size(), 0);
  for(int y = 0; y < bg.rows; ++y)
    for(int x = 0; x < bg.cols; ++x)
      vis(y, x) = bg(y, x) * 255;
  
  cv::imwrite(getDebugPath() + "-background.png", vis);
}

cv::Mat1f BackgroundModeler::getZBuffer(const rgbd::Cloud& pcd) const
{
  cv::Mat1f zbuf(cv::Size(pcd.width, pcd.height), -1);
  for(int y = 0; y < zbuf.rows; ++y) {
    for(int x = 0; x < zbuf.cols; ++x) {
      int idx = y * zbuf.cols + x;
      if(!isnan(pcd[idx].z))
	zbuf(y, x) = pcd[idx].z;
    }
  }

  return zbuf;
}
