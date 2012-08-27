#ifndef DEPTH_DISTORTION_LEARNER_H
#define DEPTH_DISTORTION_LEARNER_H

#include <Eigen/StdVector>
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

class DepthDistortionLearner
{
public:
  std::vector< std::vector<PixelStats> > statistics_;
  
  DepthDistortionLearner(const PrimeSenseModel& initial_model);
  void addFrame(Frame frame,
		rgbd::Cloud::ConstPtr pcd,
		const Eigen::Affine3f& transform);
  PrimeSenseModel fitModel() const;
  void clear() { frames_.clear(); pcds_.clear(); transforms_.clear(); }
  
protected:
  //! Used for 3D -> 2D projection.  Depth distortion params are ignored.
  PrimeSenseModel initial_model_;
  std::vector<Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  //! transforms_[i] takes pcds_[i] to the coordinate system that frames_[i] is defined in.
  std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > transforms_;
};

#endif // DEPTH_DISTORTION_LEARNER_H
