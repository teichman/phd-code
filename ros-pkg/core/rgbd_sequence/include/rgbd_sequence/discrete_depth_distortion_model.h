#ifndef DISCRETE_DEPTH_DISTORTION_MODEL_H
#define DISCRETE_DEPTH_DISTORTION_MODEL_H

#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_sequence/primesense_model.h>
#include <bag_of_tricks/lockable.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>

class DepthDistortionModel : public Serializable
{
public:
  DepthDistortionModel() {}
  virtual ~DepthDistortionModel() {}

  virtual void undistort(rgbd::Frame* frame) const = 0;
};

//! Models the depth multiplier in this block as \exp(\alpha z + \beta).
class ExponentialFrustum : public Serializable, public SharedLockable
{
public:
  ExponentialFrustum();
  void fitParams(const std::vector<double>& ground_truth, const std::vector<double>& measurements);
  void undistort(double* z) const;
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);

protected:
  double alpha_;
  double beta_;
};

class ExponentialDepthDistortionModel : public DepthDistortionModel
{
public:
  ExponentialDepthDistortionModel();
  ~ExponentialDepthDistortionModel();

  void undistort(rgbd::Frame* frame) const;
};

class DiscreteFrustum : public Serializable, public SharedLockable
{
public:
  DiscreteFrustum(int smoothing = 1, double bin_depth = 1.0);
  //! z value, not distance to origin.
  void addExample(double ground_truth, double measurement);
  int index(double z) const;
  void undistort(double* z) const;
//  void undistort(int idx, float* z, float* mult) const;
  void interpolatedUndistort(double* z) const;
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  
protected:
  double max_dist_;
  int num_bins_;
  double bin_depth_;
  Eigen::VectorXf counts_;
  // Eigen::VectorXf total_multipliers_;
  Eigen::VectorXf total_numerators_;
  Eigen::VectorXf total_denominators_;
  Eigen::VectorXf multipliers_;

  friend class DiscreteDepthDistortionModel;
};

class DiscreteDepthDistortionModel : public DepthDistortionModel
{
public:
  typedef boost::shared_ptr<DiscreteDepthDistortionModel> Ptr;
  typedef boost::shared_ptr<const DiscreteDepthDistortionModel> ConstPtr;
  DiscreteDepthDistortionModel() {}
  ~DiscreteDepthDistortionModel();
  DiscreteDepthDistortionModel(const rgbd::PrimeSenseModel& psm, int bin_width = 8, int bin_height = 6, double bin_depth = 2.0, int smoothing = 1);
  DiscreteDepthDistortionModel(const DiscreteDepthDistortionModel& other);
  DiscreteDepthDistortionModel& operator=(const DiscreteDepthDistortionModel& other);
  void undistort(rgbd::Frame* frame) const;
  //! Returns the number of training examples it used from this pair.
  size_t accumulate(const rgbd::Frame& ground_truth, const rgbd::Frame& measurement);
  void addExample(const rgbd::ProjectivePoint& ppt, double ground_truth, double measurement);
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  //! Saves images to the directory found at path.
  //! If path doesn't exist, it will be created.
  void visualize(const std::string& path) const;
  
protected:
  rgbd::PrimeSenseModel psm_;
  int width_;
  int height_;
  int bin_width_;
  int bin_height_;
  double bin_depth_;
  int num_bins_x_;
  int num_bins_y_;
  // Gross.  I did it.  I used mutable.  This seems like a legitimate use though; the caches change
  // when you call undistort(), but the actual distortion model isn't changing, and that's really what
  // we want to capture with the idea of constness.
  mutable Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> idx_cache_;
  mutable Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> multiplier_cache_;

  //! frustums_[y][x]
  std::vector< std::vector<DiscreteFrustum*> > frustums_;

  void deleteFrustums();
  //! depth is in meters
  DiscreteFrustum& frustum(int y, int x);
  const DiscreteFrustum& frustum(int y, int x) const;
};

#endif // DISCRETE_DEPTH_DISTORTION_MODEL_H
