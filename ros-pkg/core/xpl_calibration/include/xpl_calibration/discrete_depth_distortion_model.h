#ifndef DISCRETE_DEPTH_DISTORTION_MODEL_H
#define DISCRETE_DEPTH_DISTORTION_MODEL_H

#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_sequence/primesense_model.h>
#include <bag_of_tricks/lockable.h>

#define MAX_MULT 1.2
#define MIN_MULT 0.8

class Frustum : public Serializable, public SharedLockable
{
public:
  Frustum(int smoothing = 1, double bin_depth = 1.0);
  //! z value, not distance to origin.
  void addExample(double ground_truth, double measurement);
  void addMultiplier(double measurement, double multiplier);
  void undistort(double* z) const;
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  
protected:
  double max_dist_;
  int num_bins_;
  double bin_depth_;
  Eigen::VectorXf counts_;
  Eigen::VectorXf total_multipliers_;
  Eigen::VectorXf multipliers_;

  friend class DiscreteDepthDistortionModel;
};

class DiscreteDepthDistortionModel : public Serializable
{
public:
  DiscreteDepthDistortionModel() {}
  ~DiscreteDepthDistortionModel();
  DiscreteDepthDistortionModel(const rgbd::PrimeSenseModel& psm, int bin_width = 4, int bin_height = 3, double bin_depth = 0.25, int smoothing = 1);
  void undistort(rgbd::Frame* frame) const;
  void accumulate(const rgbd::Frame& ground_truth, const rgbd::Frame& measurement);
  void accumulate(const rgbd::Frame& measurement, const Eigen::MatrixXd& multipliers);
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

  //! frustums_[y][x]
  std::vector< std::vector<Frustum*> > frustums_;

  void deleteFrustums();
  //! depth is in meters
  Frustum& frustum(int y, int x);
  const Frustum& frustum(int y, int x) const;
};

#endif // DISCRETE_DEPTH_DISTORTION_MODEL_H
