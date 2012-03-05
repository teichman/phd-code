#ifndef BACKGROUND_MODELER_H
#define BACKGROUND_MODELER_H

#include <xpl_calibration/common.h>

class DepthHistogram
{
public:
  typedef boost::shared_ptr<DepthHistogram> Ptr;

  DepthHistogram(double minval, double maxval, double binwidth);
  void initialize(double minval, double maxval, double binwidth);
  void insert(double val);
  void clear();
  size_t total() const { return total_; }
  size_t getCounts(size_t idx) const { return bins_[idx]; }
  
protected:
  double minval_;
  double maxval_;
  double binwidth_;
  std::vector<double> lower_limits_;
  std::vector<size_t> bins_;
  size_t total_;
};

class BackgroundModel
{
public:
  virtual ~BackgroundModel() {}
  virtual bool isBackground(size_t idx, double z) const = 0;
  virtual size_t size() const = 0;
};

class HistogramBackgroundModeler : public pipeline::Pod, public BackgroundModel
{
public:
  typedef bool (*QueryFunction)(size_t, double);
  
  DECLARE_POD(HistogramBackgroundModeler);
  HistogramBackgroundModeler(std::string name) :
    Pod(name)
  {
    declareParam<double>("Resolution", 0.1); // meters
    declareParam<double>("MaxDepth", 15); 
    declareParam<double>("MinPercent", 0.2); // [0, 1]
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    declareOutput<const BackgroundModel*>("BackgroundModel");
  }

  bool isBackground(size_t idx, double z) const;
  void compute();
  void debug() const;
  size_t size() const { return histograms_.size(); }

protected:
  std::vector<DepthHistogram::Ptr> histograms_; // row major

  cv::Mat1f getZBuffer(const rgbd::Cloud& pcd) const;
};

#endif // BACKGROUND_MODELER_H
