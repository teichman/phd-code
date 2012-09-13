#ifndef DEPTH_DISTORTION_LEARNER_H
#define DEPTH_DISTORTION_LEARNER_H

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
  CoverageMap(int rows, int cols);
  void addFrame(rgbd::Frame frame);
  cv::Mat3b computeImage() const;

protected:
  std::vector< std::vector< std::vector<double> > > bins_;
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
  void clear() { frames_.clear(); pcds_.clear(); transforms_.clear(); }
  size_t size() const;
  
protected:
  //! Used for 3D -> 2D projection.  Depth distortion params are ignored.
  rgbd::PrimeSenseModel initial_model_;
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  //! transforms_[i] takes pcds_[i] to the coordinate system that frames_[i] is defined in.
  std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > transforms_;
};

#endif // DEPTH_DISTORTION_LEARNER_H
