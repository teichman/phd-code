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
  int getNumNearby(double z) const;
  void finalize();
    
protected:
  double minval_;
  double maxval_;
  double binwidth_;
  std::vector<double> lower_limits_;
  std::vector<size_t> bins_;
  std::vector<size_t> num_nearby_;
  size_t total_;
};

class BackgroundModel
{
public:
  virtual ~BackgroundModel() {}
  virtual bool isBackground(size_t idx, double z) const = 0;
  virtual size_t size() const = 0;
};

class HistogramBackgroundModeler : public pl::Pod, public BackgroundModel
{
public:
  typedef bool (*QueryFunction)(size_t, double);
  
  DECLARE_POD(HistogramBackgroundModeler);
  HistogramBackgroundModeler(std::string name) :
    Pod(name)
  {
    declareParam<double>("Resolution", 0.1); // meters
    declareParam<double>("MaxDepth", 10);
    declareParam<int>("Stride", 1); // Use every kth pointcloud for building the model.
    declareParam<double>("MinPercent", 0.2); // [0, 1]
    declareInput<StreamConstPtr>("Sequence");
    declareOutput<const BackgroundModel*>("BackgroundModel");
  }

  bool isBackground(size_t idx, double z) const;
  void compute();
  void debug() const;
  size_t size() const { return histograms_.size(); }

protected:
  std::vector<DepthHistogram::Ptr> histograms_; // row major
  double min_pct_;
    
  cv::Mat1f getZBuffer(const rgbd::Cloud& pcd) const;
};

#endif // BACKGROUND_MODELER_H
