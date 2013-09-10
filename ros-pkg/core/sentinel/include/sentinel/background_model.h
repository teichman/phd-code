#ifndef BACKGROUND_MODEL_H
#define BACKGROUND_MODEL_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <OpenNI.h>
#include <opencv2/imgproc/imgproc.hpp>

#define MAX_DEPTH 5
#define MIN_DEPTH 0.5

class DepthHistogram;

class BackgroundModel
{
public:
  BackgroundModel(int width, int height,
                  int width_step, int height_step,
                  double min_pct, double max_depth,
                  double bin_width);
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
  
protected:
  int width_;
  int height_;
  int width_step_;
  int height_step_;
  int blocks_per_row_;
  int blocks_per_col_;
  std::vector<DepthHistogram> histograms_; // row major
  //! Percentage of histogram that a bin must contain to count as background.
  double min_pct_;
  double max_depth_;
  //! Bin width in z.
  double bin_width_;
  cv::Mat1b block_img_;
  cv::Mat1b dilated_block_img_;
};

class DepthHistogram
{
public:
  DepthHistogram(double min_depth, double max_depth, double binwidth);
  void initialize(double min_depth, double max_depth, double binwidth);
  void increment(double z, int num);
  void clear();
  double total() const { return total_; }

  // Inline for speed.
  double getNum(double z) const
  {
    size_t lower_idx;
    double upper_weight;
    indices(z, &lower_idx, &upper_weight);
    return bins_[lower_idx] * (1.0 - upper_weight) + bins_[lower_idx + 1] * (upper_weight);
  }

  // Inline for speed.
  void indices(double z, size_t* lower_idx, double* upper_weight) const
  {
    *lower_idx = std::max<size_t>(0, (z - min_depth_) * inv_binwidth_);
    *upper_weight = (z - lower_limits_[*lower_idx]) * inv_binwidth_;
  }
               
  
protected:
  double min_depth_;
  double max_depth_;
  double binwidth_;
  double inv_binwidth_;
  std::vector<double> lower_limits_;
  std::vector<double> bins_;
  double total_;
};


#endif // BACKGROUND_MODEL_H
