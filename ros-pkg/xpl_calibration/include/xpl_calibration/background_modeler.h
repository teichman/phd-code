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
  void getBackground(int minpts, double* minval, double* maxval) const;
  
protected:
  double minval_;
  double maxval_;
  double binwidth_;
  std::vector<double> lower_limits_;
  std::vector<int> bins_;
};

class BackgroundModeler : public pipeline::Pod
{
public:
  DECLARE_POD(BackgroundModeler);
  BackgroundModeler(std::string name) :
    Pod(name)
  {
    declareParam<double>("Resolution", 0.25); // meters
    declareParam<double>("MaxDepth", 10.0); 
    declareParam<int>("MinNumPoints", 20);
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    // Points between min and max are background.
    declareOutput<const std::vector<double>*>("MinDistances");
    declareOutput<const std::vector<double>*>("MaxDistances");
  }

  void compute();
  void debug() const;

protected:
  std::vector<DepthHistogram::Ptr> histograms_; // row major
  std::vector<double> min_distances_;
  std::vector<double> max_distances_;

  cv::Mat1f getZBuffer(const rgbd::Cloud& pcd) const;
};

#endif // BACKGROUND_MODELER_H
