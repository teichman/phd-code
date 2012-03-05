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

  ++bins_[idx];
  ++total_;
}

void DepthHistogram::clear()
{
  minval_ = -1;
  maxval_ = -1;
  binwidth_ = -1;
  lower_limits_.clear();
  bins_.clear();
  total_ = 0;
}  

void HistogramBackgroundModeler::compute()
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
  double pct = (double)histograms_[idx]->getCounts(idx) / (double)histograms_[idx]->total();
  if(pct > param<double>("MinPercent"))
    return true;
  else
    return false;
}
