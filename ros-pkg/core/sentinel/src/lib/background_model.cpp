#include <sentinel/background_model.h>
#include <ros/assert.h>
#include <timer/timer.h>

using namespace std;

DepthHistogram::DepthHistogram(double min_depth, double max_depth, double binwidth) 
{
  initialize(min_depth, max_depth, binwidth);
}

void DepthHistogram::initialize(double min_depth, double max_depth, double binwidth)
{
  clear();

  total_ = 0;
  min_depth_ = min_depth;
  max_depth_ = max_depth;
  binwidth_ = binwidth;
  inv_binwidth_ = 1.0 / binwidth_;

  int num_bins = ceil((max_depth_ - min_depth_) / binwidth_);
  bins_.resize(num_bins, 0);
  lower_limits_.resize(num_bins);
  for(int i = 0; i < num_bins; ++i)
    lower_limits_[i] = min_depth_ + i * binwidth_;
}

void DepthHistogram::increment(double z, int num)
{
  size_t lower_idx;
  double upper_weight;
  indices(z, &lower_idx, &upper_weight);
  
  total_ += num;
  bins_[lower_idx] += num * (1.0 - upper_weight);
  bins_[lower_idx+1] += num * upper_weight;
}

void DepthHistogram::clear()
{
  min_depth_ = -1;
  max_depth_ = -1;
  binwidth_ = -1;
  inv_binwidth_ = -1;
  lower_limits_.clear();
  bins_.clear();
  total_ = 0;
}

BackgroundModel::BackgroundModel(int width, int height,
                                 int width_step, int height_step,
                                 double min_pct, double max_depth,
                                 double bin_width) :
  width_(width),
  height_(height),
  width_step_(width_step),
  height_step_(height_step),
  min_pct_(min_pct),
  max_depth_(max_depth),
  bin_width_(bin_width)
{
  ROS_ASSERT(height_step_ % 2 == 0 && width_step_ % 2 == 0);
  ROS_ASSERT(height_ % height_step_ == 0 && width_ % width_step_ == 0);

  blocks_per_row_ = width_ / width_step_;
  blocks_per_col_ = height_ / height_step_;
  
  size_t num = 0;
  for(int y = height_step_ / 2; y < height; y += height_step_)
    for(int x = width_step_ / 2; x < width; x += width_step_)
      ++num;
  histograms_.resize(num, DepthHistogram(0, max_depth_, bin_width_));
  cout << "Initialized " << histograms_.size() << " histograms." << endl;
}

void BackgroundModel::increment(openni::VideoFrameRef depth, int num)
{
  #if JARVIS_DEBUG
  ScopedTimer st("BackgroundModel::increment");
  #endif

  ROS_ASSERT(depth.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM);
  uint16_t* data = (uint16_t*)depth.getData();
  size_t idx = 0;
  for(int y = height_step_ / 2; y < height_; y += height_step_)
    for(int x = width_step_ / 2; x < width_; x += width_step_, ++idx)
      histograms_[idx].increment(data[y * depth.getWidth() + x] * 0.001, num);
}

size_t BackgroundModel::predict(openni::VideoFrameRef depth, vector<uint8_t>* mask) const
{
  ROS_ASSERT(depth.getWidth() * depth.getHeight() == (int)mask->size());
  ROS_ASSERT(width_ == depth.getWidth());
  
  size_t idx = 0;
  size_t num = 0;
  uint16_t* data = (uint16_t*)depth.getData();

  for(int y = height_step_ / 2; y < height_; y += height_step_) {
    for(int x = width_step_ / 2; x < width_; x += width_step_, ++idx) {
      uint16_t val = data[y * width_ + x];
      if(val == 0)
        continue;
      
      double pct = histograms_[idx].getNum(val * .001) / histograms_[idx].total();
      if(pct < min_pct_) {
        int r = idx / blocks_per_row_;
        int c = idx - r * blocks_per_row_;
        for(int y2 = r * height_step_; y2 < (r+1) * height_step_; ++y2) {
          for(int x2 = c * width_step_; x2 < (c+1) * width_step_; ++x2) {
            (*mask)[y2 * width_ + x2] = 255;
            ++num;
          }
        }
      }
    }
  }
  return num;
}

