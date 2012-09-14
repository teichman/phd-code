#ifndef DEPTH_DISTORTION_LEARNER_H
#define DEPTH_DISTORTION_LEARNER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/StdVector>
#include <pcl/common/transforms.h>
#include <rgbd_sequence/primesense_model.h>
#include <optimization/optimization.h>
#include <xpl_calibration/mean_depth_error.h>

class PixelStats
{
public:
  std::vector<double> velo_;
  std::vector<double> asus_;

  void addPoint(double velo, double asus);
  void stats(double* mean, double* stdev, double* num) const;
  bool valid() const;
  void reserve(int num) { velo_.reserve(num); asus_.reserve(num); }
};

class CoverageMap
{
public:
  double min_dist_;
  double max_dist_;
  
  CoverageMap(double scale, int rows, int cols);
  void addFrame(rgbd::Frame frame);
  cv::Mat3b computeImage() const;
  size_t rows() const { return bins_.size(); }
  size_t cols() const { return bins_[0].size(); }
  void clear();
  
protected:
  //! What to multiply the original by.
  double scale_;
  //! In the original, not scaled down.
  int rows_;
  //! In the original, not scaled down.
  int cols_;
  std::vector< std::vector< std::vector<double> > > bins_;
  
  cv::Vec3b colorizeBin(const std::vector<double>& bin) const;
};

class DepthDistortionLearner
{
public:
  //! Used for 3D -> 2D projection.
  rgbd::PrimeSenseModel initial_model_;

  DepthDistortionLearner(const rgbd::PrimeSenseModel& initial_model);
  //! pcd is assumed to be in the same coordinate system as frame, i.e.
  //! model.frameToCloud(frame) should in theory produce pcd.
  void addFrame(rgbd::Frame frame, rgbd::Cloud::ConstPtr pcd);
  rgbd::PrimeSenseModel fitModel();
  rgbd::PrimeSenseModel fitFocalLength();
  void clear() { frames_.clear(); pcds_.clear(); coverage_map_.clear(); }
  size_t size() const;
  cv::Mat3b coverageMap() const { return coverage_map_.computeImage(); }
  
protected:
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  CoverageMap coverage_map_;

  Eigen::VectorXd regress(const Eigen::MatrixXd& X, const Eigen::VectorXd& Y) const;
  Eigen::VectorXd regressRegularized(const Eigen::MatrixXd& X, const Eigen::VectorXd& Y) const;
};


//! 1/2 (X^T w - Y)^T (X^T w - Y) + 1/2 \gamma w^T w
class RegularizedRegressionObjective : public ScalarFunction
{
public:
  RegularizedRegressionObjective(double gamma, const Eigen::MatrixXd* X, const Eigen::VectorXd* Y);
  double eval(const Eigen::VectorXd& x) const;
  
protected:
  double gamma_;
  const Eigen::MatrixXd* X_;
  const Eigen::VectorXd* Y_;
};

//! \gamma w + X (X^T w - Y)
class RegularizedRegressionGradient : public VectorFunction
{
public:
  RegularizedRegressionGradient(double gamma, const Eigen::MatrixXd* X, const Eigen::VectorXd* Y);
  Eigen::VectorXd eval(const Eigen::VectorXd& x) const;
  
protected:
  double gamma_;
  const Eigen::MatrixXd* X_;
  const Eigen::VectorXd* Y_;
};


#endif // DEPTH_DISTORTION_LEARNER_H
