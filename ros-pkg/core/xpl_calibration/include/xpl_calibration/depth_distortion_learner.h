#ifndef DEPTH_DISTORTION_LEARNER_H
#define DEPTH_DISTORTION_LEARNER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/StdVector>
#include <pcl/common/transforms.h>
#include <rgbd_sequence/primesense_model.h>

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
  //int max_tr_ex_;
  
  DepthDistortionLearner(const rgbd::PrimeSenseModel& initial_model);
  void addFrame(rgbd::Frame frame,
		rgbd::Cloud::ConstPtr pcd,
		const Eigen::Affine3f& transform);
  rgbd::PrimeSenseModel fitModel();
  void clear() { frames_.clear(); pcds_.clear(); transforms_.clear(); coverage_map_.clear(); }
  size_t size() const;
  cv::Mat3b coverageMap() const { return coverage_map_.computeImage(); }
  
protected:
  //! Used for 3D -> 2D projection.  Depth distortion params are ignored.
  rgbd::PrimeSenseModel initial_model_;
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  //! transforms_[i] takes pcds_[i] to the coordinate system that frames_[i] is defined in.
  std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > transforms_;
  CoverageMap coverage_map_;
};

#endif // DEPTH_DISTORTION_LEARNER_H
