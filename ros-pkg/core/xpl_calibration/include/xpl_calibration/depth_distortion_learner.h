#ifndef DEPTH_DISTORTION_LEARNER_H
#define DEPTH_DISTORTION_LEARNER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/StdVector>
#include <pcl/common/transforms.h>
#include <rgbd_sequence/primesense_model.h>
#include <optimization/optimization.h>
#include <bag_of_tricks/lockable.h>
#include <xpl_calibration/mean_depth_error.h>
#include <xpl_calibration/discrete_depth_distortion_model.h>

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

class CoverageMap2 : public Lockable
{
public:
  //! Min number to be considered filled.
  int min_counts_;
  
  CoverageMap2(int rows, int cols, double min_dist, double max_dist, int num_bins);
  void increment(int v, int u, double dist);
  void saveVisualizations(const std::string& dir) const;
  int rows() const { return counts_[0].rows(); }
  int cols() const { return counts_[0].cols(); }
  
protected:
  double min_dist_;
  double max_dist_;
  std::vector<Eigen::MatrixXi> counts_;
  double width_;

  cv::Mat3b visualizeSlice(const Eigen::MatrixXi& counts) const;
};

class DepthDistortionLearner
{
public:
  //! Used for 3D -> 2D projection.
  rgbd::PrimeSenseModel initial_model_;
  //! With velodyne data, this should be false.
  //! This is for dirty slam data.
  bool use_filters_;

  DepthDistortionLearner(const rgbd::PrimeSenseModel& initial_model);
  //! pcl::transformPointCloud(*pcd, *pcd, transform) should put pcd into frame's coordinate system.
  //! For slam calibration, pcds are just pointers to the map.
  //! For velo calibration, each pcd is a separate velo pointcloud.
  void addFrame(rgbd::Frame frame, rgbd::Cloud::ConstPtr pcd, const Eigen::Affine3d& transform);
  DiscreteDepthDistortionModel fitDiscreteModel();
  rgbd::PrimeSenseModel fitModel();
  rgbd::PrimeSenseModel fitFocalLength();
  void clear() { frames_.clear(); pcds_.clear(); transforms_.clear(); coverage_map_.clear(); }
  size_t size() const;
  cv::Mat3b coverageMap() const { return coverage_map_.computeImage(); }
  
protected:
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  std::vector<Eigen::Affine3d> transforms_;
  CoverageMap coverage_map_;

  Eigen::VectorXd regress(const Eigen::MatrixXd& X, const Eigen::VectorXd& Y) const;
  Eigen::VectorXd regressRegularized(const Eigen::MatrixXd& X, const Eigen::VectorXd& Y) const;
  void computeMultiplierMap(const rgbd::PrimeSenseModel& model,
			    const rgbd::DepthMat& depth,
			    const rgbd::DepthMat& mapdepth,
			    const rgbd::DepthIndex& dindex,
			    Eigen::MatrixXd* multipliers,
			    cv::Mat3b* visualization) const;
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
