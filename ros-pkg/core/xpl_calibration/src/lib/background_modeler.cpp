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

  total_ = 0;
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

  ++total_;
  ++bins_[idx];
}

void DepthHistogram::clear()
{
  minval_ = -1;
  maxval_ = -1;
  binwidth_ = -1;
  lower_limits_.clear();
  bins_.clear();
  num_nearby_.clear();
  total_ = 0;
}

void DepthHistogram::finalize()
{
  num_nearby_.resize(bins_.size());
  for(int idx = 0; idx < (int)num_nearby_.size(); ++idx)
    for(int i = idx - 1; i <= idx + 1; ++i)
      if(i >= 0 && i < (int)bins_.size())
        num_nearby_[idx] = max(bins_[i], num_nearby_[idx]);
}

int DepthHistogram::getNumNearby(double z) const
{
  ROS_ASSERT(num_nearby_.size() == bins_.size());
  int idx = (z - minval_) / binwidth_;
  if(idx < 0 || idx >= (int)bins_.size())
    return -1;
  else
    return num_nearby_[idx];
}

void HistogramBackgroundModeler::compute()
{
  // -- Initialize.
  const Stream& strm = *pull<StreamConstPtr>("Sequence");
  ROS_ASSERT(strm[0]->isOrganized());
  size_t num_pixels = strm[0]->size();
  min_pct_ = param<double>("MinPercent"); // The param call is slow.
  
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
  for(size_t i = 0; i < strm.size(); i += param<int>("Stride")) {
    std::cout << "Accumulating cloud " << i+1 << " / " << strm.size() << std::endl;
    const Cloud& pcd = *strm[i];
    for(size_t j = 0; j < pcd.size(); ++j) {
      if(pcl_isfinite(pcd[j].z))
        histograms_[j]->insert(pcd[j].z);
    }
  }
  hrt.stop();
  cout << hrt.report() << endl;

  for(size_t i = 0; i < histograms_.size(); ++i)
    histograms_[i]->finalize();
  
  push<const BackgroundModel*>("BackgroundModel", this);
}

void HistogramBackgroundModeler::debug() const
{
  
}

cv::Mat1f HistogramBackgroundModeler::getZBuffer(const rgbd::Cloud& pcd) const
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

bool HistogramBackgroundModeler::isBackground(size_t idx, double z) const
{
  ROS_ASSERT(idx < histograms_.size());

  int num = histograms_[idx]->getNumNearby(z);
  if(num == -1)
    return true;
    
  double pct = (double)num / (double)histograms_[idx]->total();
  if(pct > min_pct_)
    return true;
  else
    return false;
}
