#ifndef BACKGROUND_MODEL_H
#define BACKGROUND_MODEL_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>

typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> DepthMat;  
typedef boost::shared_ptr<DepthMat> DepthMatPtr;
typedef boost::shared_ptr<const DepthMat> DepthMatConstPtr;

class DepthHistogram
{
public:
  typedef boost::shared_ptr<DepthHistogram> Ptr;

  DepthHistogram(double min_depth, double max_depth, double binwidth);
  void initialize(double min_depth, double max_depth, double binwidth);
  void increment(double z, int num);
  void clear();
  size_t total() const { return total_; }
  int getNumNearby(double z) const;
  void finalize();
    
protected:
  double min_depth_;
  double max_depth_;
  double binwidth_;
  std::vector<double> lower_limits_;
  std::vector<size_t> bins_;
  std::vector<size_t> num_nearby_;
  size_t total_;
  bool finalized_;
};

class BackgroundModel
{
public:
  BackgroundModel(int num_pixels, double min_pct = 0.2, double max_depth = 10, double res = 0.1);
  //! Increments bins by num.
  void increment(const DepthMat& depth, int num = 1);
  void finalize();

  bool isBackground(size_t idx, double z) const;
  size_t size() const { return histograms_.size(); }
  
protected:
  std::vector<DepthHistogram::Ptr> histograms_; // row major
  double min_pct_;
  double max_depth_;
  double res_;
  bool finalized_;
};

#endif // BACKGROUND_MODEL_H
