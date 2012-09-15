#ifndef DEPTH_DISTORTION_LEARNER_H
#define DEPTH_DISTORTION_LEARNER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/StdVector>
#include <pcl/common/transforms.h>
#include <rgbd_sequence/primesense_model.h>
#include <optimization/optimization.h>
#include <bag_of_tricks/lockable.h>
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

// //! Represents one small region of the image.
// class DescriptorAccumulatorCell
// {
// public:
//   DescriptorAccumulatorCell(double min_dist, double max_dist, int num_bins, size_t max_per_bin);
//   void addDescriptor(double distance, const VectorXd& descriptor, double multiplier);
//   cv::Vec3b color() const;
  
//   //! descriptors_[i][j] is the jth descriptor in the ith distance bin.
//   std::vector< std::vector<Eigen::VectorXd> > descriptors_;
//   std::vector< std::vector<Eigen::VectorXd> > multipliers_;

// protected:
//   double min_dist_;
//   double max_dist_;
//   int num_bins_;
//   size_t max_per_bin_;
//   double width_;
// };

// class DescriptorAccumulator
// {
// public:
//   //! rows and cols in the original.
//   //! scale is how much to scale down the original image when creating bins.
//   //! Each cell is a small subwindow in the original image.
//   //! Each cell has a set of bins over possible distances.
//   DescriptorAccumulator(int rows, int cols, double scale, int bins_per_cell,
// 			double min_dist, double max_dist, uint64_t max_bytes);
//   void addFrame(rgbd::Frame measurement, rgbd::Frame map);
//   cv::Mat3b computeCoverageImage() const;
//   Eigen::MatrixXd assembleX() const;
//   Eigen::VectorXd assembleY() const;
  
//   size_t rows() const { return bins_.size(); }
//   size_t cols() const { return bins_[0].size(); }
//   void clear();
  
// protected:
//   //! In the original, not scaled down.
//   int rows_;
//   //! In the original, not scaled down.
//   int cols_;
//   //! What to multiply the original by.
//   double scale_;
//   double min_dist_;
//   double max_dist_;
//   size_t max_per_bin_;
//   //! bins_[y][x][i] is the ith bin in the row y, col x.
//   std::vector< std::vector<DescriptorAccumulatorCell> > cells_;
//   //! Map distance values aligned with descriptors in bins_.
//   std::vector< std::vector< std::vector<double> > > ys_;
  
//   cv::Vec3b colorizeBin(const std::vector<double>& bin) const;
// };

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
  //! With velodyne data, this should be false.
  //! This is for dirty slam data.
  bool use_filters_;

  DepthDistortionLearner(const rgbd::PrimeSenseModel& initial_model);
  //! pcl::transformPointCloud(*pcd, *pcd, transform) should put pcd into frame's coordinate system.
  //! For slam calibration, pcds are just pointers to the map.
  //! For velo calibration, each pcd is a separate velo pointcloud.
  void addFrame(rgbd::Frame frame, rgbd::Cloud::ConstPtr pcd, const Eigen::Affine3d& transform);
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
