#ifndef BACKGROUND_MODEL_H
#define BACKGROUND_MODEL_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <OpenNI.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <ros/assert.h>

#define MAX_DEPTH 6
#define MIN_DEPTH 0.5

class DepthHistogram;
class OccupancyLine;

class BackgroundModel
{
public:
  BackgroundModel(int width, int height,
                  int width_step, int height_step,
                  double max_depth, double min_depth,
                  double min_bin_width,
                  double occupancy_threshold,
                  int raytracing_threshold);
  ~BackgroundModel() {
    #if JARVIS_DEBUG
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    #endif
  }

  //! Increments bins by num.
  void increment(openni::VideoFrameRef depth, int num = 1);
  void predict(openni::VideoFrameRef depth,
               std::vector<uint32_t>* indices,
               std::vector<uint32_t>* fg_markers,
               std::vector<uint32_t>* bg_fringe_markers);
  size_t size() const { return histograms_.size(); }
  int height() const { return height_; }
  int width() const { return width_; }
  int heightStep() const { return height_step_; }
  int widthStep() const { return width_step_; }
  double transform(double input) const;
  double transformDerivative(double input) const;
  double inverseTransform(double x) const;
  void debug(int x, int y);
  size_t numUpdates() const { return num_updates_; }
  
protected:
  int width_;
  int height_;
  int width_step_;
  int height_step_;
  int blocks_per_row_;
  int blocks_per_col_;
  std::vector<OccupancyLine> histograms_; // row major
  double min_depth_;
  double max_depth_;
  //! Bin width in z.
  double min_bin_width_;
  double occupancy_threshold_;
  int raytracing_threshold_;
  size_t num_updates_;
  cv::Mat1b block_img_;
  cv::Mat1b dilated_block_img_;
  Eigen::VectorXd weights_;
  cv::Mat3b vis_;
  
  void initializeWeights();
  void initializeCubicWeights();
};

class DepthHistogram
{
public:
  bool debug_;
  int x_;
  int y_;
  
  DepthHistogram(double min_depth, double max_depth, double binwidth,
                 int x, int y);
  void initialize(double min_depth, double max_depth, double binwidth);
  void increment(double z, int num);
  void clear();
  double total() const { return total_; }
  std::string status(const std::string& prefix = "") const;
  
  // Inline for speed.
  double getNum(double z) const
  {
    ROS_ASSERT(z >= min_depth_ && z < max_depth_);
    size_t lower_idx;
    double upper_weight;
    indices(z, &lower_idx, &upper_weight);
    return bins_[lower_idx] * (1.0 - upper_weight) + bins_[lower_idx + 1] * (upper_weight);
  }

  // Inline for speed.
  void indices(double z, size_t* lower_idx, double* upper_weight) const
  {
    ROS_ASSERT(z >= min_depth_ && z < max_depth_);
    *lower_idx = std::max<int>(0, (z - min_depth_) * inv_binwidth_);
    // Very rare edge case, but I have seen it in practice.
    *lower_idx = std::min<int>(lower_limits_.size() - 2, *lower_idx);
    *upper_weight = (z - lower_limits_[*lower_idx]) * inv_binwidth_;
  }

  friend class BackgroundModel;
  
protected:
  double min_depth_;
  double max_depth_;
  double binwidth_;
  double inv_binwidth_;
  //! A separate, special bin is maintained for depth dropouts.
  double dropout_count_;
  double total_;
  std::vector<double> lower_limits_;
  std::vector<double> bins_;
};


class OccupancyLine
{
public:
  bool debug_;
  int x_;
  int y_;
  
  OccupancyLine(double min_depth, double max_depth, double binwidth,
                int x, int y, int raytracing_threshold);
  void initialize(double min_depth, double max_depth, double binwidth);
  void increment(double z, int num);
  void clear();
  std::string status(const std::string& prefix = "") const;
  
  // Inline for speed.
  double getNum(double z) const
  {
    //ROS_ASSERT(z >= min_depth_ && z < max_depth_);
    size_t lower_idx;
    double upper_weight;
    indices(z, &lower_idx, &upper_weight);
    //return bins_[lower_idx] * (1.0 - upper_weight) + bins_[lower_idx + 1] * (upper_weight);
    return bins_[lower_idx] + bins_[lower_idx + 1];
  }

  // Inline for speed.
  void indices(double z, size_t* lower_idx, double* upper_weight) const
  {
    //ROS_ASSERT(z >= min_depth_ && z < max_depth_);
    *lower_idx = std::max<int>(0, (z - min_depth_) * inv_binwidth_);
    // Very rare edge case, but I have seen it in practice.
    *lower_idx = std::min<int>(lower_limits_.size() - 2, *lower_idx);
    *upper_weight = (z - lower_limits_[*lower_idx]) * inv_binwidth_;
  }

  friend class BackgroundModel;
  
protected:
  double min_depth_;
  double max_depth_;
  double binwidth_;
  double inv_binwidth_;
  std::vector<double> lower_limits_;
  std::vector<double> bins_;
  size_t recent_bin_idx_;
  int recent_bin_count_;
  int raytracing_threshold_;

  void raytrace(size_t lower_idx, double upper_weight);
};

void cluster(cv::Mat1f depth, float thresh,
             int min_pts, cv::Mat1i* assignments,
             std::vector< std::vector<int> >* indices = NULL);
void flood(cv::Mat1f depth, float thresh,
           int min_pts, const cv::Point2i& seed,
           int id, cv::Mat1i* ass,
           std::vector< std::vector<int> >* indices = NULL);

#endif // BACKGROUND_MODEL_H
