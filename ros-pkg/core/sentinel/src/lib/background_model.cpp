#include <sentinel/background_model.h>
#include <queue>
#include <ros/assert.h>
#include <timer/timer.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <bag_of_tricks/connected_components.h>

using namespace std;
using namespace Eigen;

void flood(cv::Mat1f depth, float thresh, int min_pts,
           const cv::Point2i& seed, int id, cv::Mat1i* ass,
           std::vector< std::vector<int> >* indices)
{  
  // This should probably be a class rather than a free function so that it can
  // reuse the allocated memory.
  vector<cv::Point2i> pts;
  pts.reserve(min_pts);
  pts.push_back(seed);
  
  std::queue<cv::Point2i> que;
  que.push(seed);
  (*ass)(seed) = -2;
  while(!que.empty()) {
    cv::Point2i pt = que.front();
    que.pop();

    ROS_ASSERT((*ass)(pt) == -2);
    (*ass)(pt) = id;
    float d0 = depth(pt);
    
    cv::Point2i dpt;
    for(dpt.y = -1; dpt.y <= 1; ++dpt.y) {
      for(dpt.x = -1; dpt.x <= 1; ++dpt.x) {
        cv::Point2i pt2 = pt + dpt;
        if(pt2.x < 0 || pt2.x >= ass->cols ||
           pt2.y < 0 || pt2.y >= ass->rows)
          continue;
        float d2 = depth(pt2);

        if(fabs(d0 - d2) < thresh && (*ass)(pt2) == -3) {
          que.push(pt2);
          (*ass)(pt2) = -2;
          pts.push_back(pt2);
        }
      }
    }
  }

  // If we didn't get enough points in this cluster, backtrack and remove them.
  // Otherwise, add the indices to the cluster.
  if(pts.size() < min_pts) {
    for(size_t i = 0; i < pts.size(); ++i)
      (*ass)(pts[i]) = -1;
  }
  else if(indices) {
    indices->push_back(vector<int>());
    indices->back().reserve(pts.size());
    for(size_t i = 0; i < pts.size(); ++i)
      indices->back().push_back(pts[i].y * ass->cols + pts[i].x);
  }
}

void cluster(cv::Mat1f depth, float thresh, int min_pts, cv::Mat1i* assignments,
             std::vector< std::vector<int> >* indices)
{
  if(indices)
    indices->clear();
  
  // -3    : unset
  // -2    : in queue
  // -1    : bg
  // >= 0    : cluster assignment
  ROS_ASSERT(depth.size() == assignments->size());
  *assignments = -3;
  for(int y = 0; y < assignments->rows; ++y)
    for(int x = 0; x < assignments->cols; ++x)
      if(depth(y, x) == 0)
        (*assignments)(y, x) = -1;

  int id = 0;
  for(int y = 0; y < assignments->rows; ++y) {
    for(int x = 0; x < assignments->cols; ++x) {
      if((*assignments)(y, x) == -3) {
        cv::KeyPoint center;
        cv::Point2i seed(x, y);
        flood(depth, thresh, min_pts, seed, id, assignments, indices);
        ++id;
      }
    }
  }
}


DepthHistogram::DepthHistogram(double min_depth, double max_depth, double binwidth,
                               int x, int y) :
  debug_(false),
  x_(x),
  y_(y)
{
  initialize(min_depth, max_depth, binwidth);
}

void DepthHistogram::initialize(double min_depth, double max_depth, double binwidth)
{
  clear();
  
  min_depth_ = min_depth;
  max_depth_ = max_depth;
  binwidth_ = binwidth;
  inv_binwidth_ = 1.0 / binwidth_;
  dropout_count_ = 0;
  total_ = 0;
  
  int num_bins = ceil((max_depth_ - min_depth_) / binwidth_);
  bins_.resize(num_bins, 0);
  lower_limits_.resize(num_bins);
  for(int i = 0; i < num_bins; ++i)
    lower_limits_[i] = min_depth_ + i * binwidth_;
}

void DepthHistogram::increment(double z, int num)
{
  // Count anything outside the accepted depth range as being a dropout.
  if(z < min_depth_ || z > max_depth_)
    z = 0;
  
  if(z < 1e-6) {
    dropout_count_ += num;
    total_ += num;
  }
  else {
    size_t lower_idx;
    double upper_weight;
    indices(z, &lower_idx, &upper_weight);
    
    bins_[lower_idx] += num * (1.0 - upper_weight);
    bins_[lower_idx+1] += num * upper_weight;
    total_ += num;
  }
  
  if(debug_) {
    cout << "#################### DepthHistogram::increment" << endl;
    cout << "Incremented by " << num << " at depth " << z << endl;
    cout << status() << endl;
    cout << "#################### aoeuaoeuaoeu" << endl;
  }
}

std::string DepthHistogram::status(const std::string& prefix) const
{
  ostringstream oss;
  double total = 0;
  oss << prefix << "Bins: " << endl;
  for(size_t i = 0; i < lower_limits_.size(); ++i) {
    oss << prefix << "  " << lower_limits_[i] << ": " << bins_[i] / total_ << endl;
    total += bins_[i];
  }
  oss << prefix << "Dropouts: " << dropout_count_ / total_ << endl;
  oss << prefix << "x: " << x_ << endl;
  oss << prefix << "y: " << y_ << endl;
  oss << prefix << "min_depth: " << min_depth_ << endl;
  oss << prefix << "max_depth_: " << max_depth_ << endl;
  oss << prefix << "binwidth_: " << binwidth_ << endl;
  oss << prefix << "inv_binwidth_: " << inv_binwidth_ << endl;
  oss << prefix << "total_: " << total_ << endl;

  total += dropout_count_;
  ROS_ASSERT(fabs(total - total_) < 1e-6);
  
  return oss.str();
}
  

void DepthHistogram::clear()
{
  min_depth_ = -1;
  max_depth_ = -1;
  binwidth_ = -1;
  inv_binwidth_ = -1;
  dropout_count_ = 0;
  total_ = 0;
  lower_limits_.clear();
  bins_.clear();
}

OccupancyLine::OccupancyLine(double min_depth, double max_depth, double binwidth,
                             int x, int y, int raytracing_threshold) :
  debug_(false),
  x_(x),
  y_(y),
  recent_bin_idx_(0),
  recent_bin_count_(0),
  raytracing_threshold_(raytracing_threshold)
{
  initialize(min_depth, max_depth, binwidth);
}

void OccupancyLine::initialize(double min_depth, double max_depth, double binwidth)
{
  clear();
  
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

void OccupancyLine::increment(double z, int num)
{
  if(z < 1e-3)
    return;
  
  z = max(z, min_depth_);
  z = min(z, max_depth_ - 1e-6);
  
  size_t lower_idx;
  double upper_weight;
  indices(z, &lower_idx, &upper_weight);
  
  bins_[lower_idx] += num * (1.0 - upper_weight);
  bins_[lower_idx+1] += num * upper_weight;
  
  if(fabs((int)lower_idx - (int)recent_bin_idx_) < 2) {
    ++recent_bin_count_;
    recent_bin_idx_ = lower_idx;
  }
  else {
    recent_bin_count_ = 0;
    recent_bin_idx_ = lower_idx;
  }
  
  if(recent_bin_count_ == raytracing_threshold_) {
    raytrace(lower_idx, upper_weight);
    recent_bin_count_ = 0;
  }
  
  if(debug_) {
    cout << "#################### OccupancyLine::increment" << endl;
    cout << "Incremented by " << num << " at depth " << z << endl;
    cout << status() << endl;
    cout << "#################### aoeuaoeuaoeu" << endl;
  }
}

void OccupancyLine::raytrace(size_t lower_idx, double upper_weight)
{
  double mx = 0;
  for(int i = 0; i < (int)lower_idx - 2; ++i) {
    mx = max(mx, bins_[i]);
    bins_[i] = 0;
  }

  bins_[lower_idx] = max(bins_[lower_idx], mx * (1.0 - upper_weight));
  bins_[lower_idx+1] = max(bins_[lower_idx+1], mx * upper_weight);
}

std::string OccupancyLine::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "Bins: " << endl;
  for(size_t i = 0; i < lower_limits_.size(); ++i) {
    if(bins_[i] > 0)
      oss << prefix << "  " << lower_limits_[i] << ": " << bins_[i] << endl;
  }
  oss << prefix << "x: " << x_ << endl;
  oss << prefix << "y: " << y_ << endl;
  oss << prefix << "min_depth: " << min_depth_ << endl;
  oss << prefix << "max_depth_: " << max_depth_ << endl;
  oss << prefix << "binwidth_: " << binwidth_ << endl;
  oss << prefix << "inv_binwidth_: " << inv_binwidth_ << endl;
  oss << prefix << "recent_bin_idx_: " << recent_bin_idx_ << endl;
  oss << prefix << "recent_bin_count_: " << recent_bin_count_ << endl;

  return oss.str();
}
  

void OccupancyLine::clear()
{
  min_depth_ = -1;
  max_depth_ = -1;
  binwidth_ = -1;
  inv_binwidth_ = -1;
  recent_bin_count_ = 0;
  lower_limits_.clear();
  bins_.clear();
}


BackgroundModel::BackgroundModel(int width, int height,
                                 int width_step, int height_step,
                                 double min_pct,
                                 double min_depth, double max_depth,
                                 double bin_width,
                                 double occupancy_threshold,
                                 int raytracing_threshold) :
  width_(width),
  height_(height),
  width_step_(width_step),
  height_step_(height_step),
  min_pct_(min_pct),
  min_depth_(min_depth),
  max_depth_(max_depth),
  bin_width_(bin_width),
  occupancy_threshold_(occupancy_threshold),
  raytracing_threshold_(raytracing_threshold),
  num_updates_(0)
{
  // -- Set up the space transform.
  // f(x) = ax^2 + bx + c
  // f'(x) = 2ax + b
  // Constraints:
  // f(0.5) = 0.5
  // f(5) = 5
  // mult * f'(5) = f'(0.5)
  double mult = 5;
  MatrixXd A(3, 3);
  A << 0.25, 0.5, 1,
    25, 5, 1,
    mult * 10 - 1, mult - 1, 0;
  VectorXd b(3);
  b << 0.5, 5, 0;
  weights_ = A.colPivHouseholderQr().solve(b);

  ROS_ASSERT(fabs(0.5 - transform(0.5)) < 1e-6);
  ROS_ASSERT(fabs(5 - transform(5)) < 1e-6);
  ROS_ASSERT(fabs(mult*transformDerivative(5) - transformDerivative(0.5)) < 1e-6);

  //ROS_ASSERT(height_step_ == 1 || height_step_ % 2 == 0);
  //ROS_ASSERT(width_step_ == 1 || width_step_ % 2 == 0);
  ROS_ASSERT(height_ % height_step_ == 0 && width_ % width_step_ == 0);

  blocks_per_row_ = width_ / width_step_;
  blocks_per_col_ = height_ / height_step_;
  block_img_ = cv::Mat1b(cv::Size(blocks_per_col_, blocks_per_row_), 0);
  dilated_block_img_ = cv::Mat1b(cv::Size(blocks_per_col_, blocks_per_row_), 0);
  
  size_t num = 0;
  for(int y = height_step_ / 2; y < height; y += height_step_)
    for(int x = width_step_ / 2; x < width; x += width_step_)
      ++num;
  histograms_.reserve(num);
  for(int y = height_step_ / 2; y < height; y += height_step_)
    for(int x = width_step_ / 2; x < width; x += width_step_)
      histograms_.push_back(OccupancyLine(min_depth_, max_depth_, bin_width_, x, y, raytracing_threshold_));

  cout << "Initialized " << histograms_.size() << " histograms." << endl;

  // -- Print out bin widths.
  const OccupancyLine& hist = histograms_[0];
  for(size_t i = 0; i < hist.lower_limits_.size(); ++i)
    cout << "Bin " << i << ": " << inverseTransform(hist.lower_limits_[i]) << endl;
}

void BackgroundModel::increment(openni::VideoFrameRef depth, int num)
{
  #if JARVIS_DEBUG
  ScopedTimer st("BackgroundModel::increment");
  #endif

  ROS_ASSERT(depth.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM);
  uint16_t* data = (uint16_t*)depth.getData();
  size_t idx = 0;
  for(int y = height_step_ / 2; y < height_; y += height_step_) {
    for(int x = width_step_ / 2; x < width_; x += width_step_, ++idx) {
      ROS_ASSERT(depth.getWidth() > 0 && depth.getHeight() > 0);
      ROS_ASSERT(depth.getDataSize() > 0);
      ROS_ASSERT(y * depth.getWidth() + x < depth.getDataSize());
      double z = data[y * depth.getWidth() + x] * 0.001;
      histograms_[idx].increment(transform(z), num);
    }
  }

  ++num_updates_;
}

void BackgroundModel::predict(openni::VideoFrameRef depth,
                              vector<uint32_t>* indices,
                              vector<uint32_t>* fg_markers,
                              vector<uint32_t>* bg_fringe_markers)
{
  ROS_ASSERT(width_ == depth.getWidth());

  indices->clear();
  fg_markers->clear();
  bg_fringe_markers->clear();
  
  size_t idx = 0;
  uint16_t* data = (uint16_t*)depth.getData();

  // double thresh = min(occupancy_threshold_, (double)numUpdates() / 2);
  // if(thresh == occupancy_threshold_) {
  //   ROS_DEBUG_ONCE("Burn-in complete.");
  // }
  
  // -- Fill in block_img_ with foreground points.
  block_img_ = 0;
  for(int y = height_step_ / 2; y < height_; y += height_step_) {
    for(int x = width_step_ / 2; x < width_; x += width_step_, ++idx) {
      uint16_t val = data[y * width_ + x];
      if(val == 0)
        continue;

      double z = val * 0.001;
      if(MIN_DEPTH > z || z > MAX_DEPTH)
        continue;
      
      if(histograms_[idx].getNum(transform(z)) < occupancy_threshold_) {
        int r = idx / blocks_per_row_;
        int c = idx - r * blocks_per_row_;
        block_img_(r, c) = 255;
      }
    }
  }

  // -- Run clustering on FG markers.  Eliminate those that are
  //    too small.
  cv::Mat1f block_depth(block_img_.size(), 0);
  for(int r = 0; r < block_img_.rows; ++r) {
    for(int c = 0; c < block_img_.cols; ++c) {
      int y = r * height_step_ + height_step_ / 2;
      int x = c * width_step_ + width_step_ / 2;
      if(block_img_(r, c) == 255) 
        block_depth(r, c) = data[y * width_ + x] * 0.001;
    }
  }
  cv::Mat1i ass(block_depth.size());
  cluster(block_depth, 0.5, 20, &ass);
  for(int r = 0; r < block_img_.rows; ++r)
    for(int c = 0; c < block_img_.cols; ++c)
      if(ass(r, c) == -1)
        block_img_(r, c) = 0;

  // if(visualize_) {
  //   cv::Mat3b clustering_scaled;
  //   cv::resize(colorAssignments(ass), clustering_scaled, ass.size(), cv::INTER_NEAREST);
  //   cv::imshow("Clustering", clustering_scaled);
  // }
  
  // -- Fill fg_markers and indices with the remaining.
  for(int r = 0; r < block_img_.rows; ++r) {
    for(int c = 0; c < block_img_.cols; ++c) {
      if(block_img_(r, c) == 255) {
        int y = r * height_step_ + height_step_ / 2;
        int x = c * width_step_ + width_step_ / 2;
        fg_markers->push_back(y * width_ + x);
        for(int y2 = r * height_step_; y2 < (r+1) * height_step_; ++y2)
          for(int x2 = c * width_step_; x2 < (c+1) * width_step_; ++x2)
            indices->push_back(y2 * width_ + x2);
      }
    }
  }
  
  // -- Fill in background fringe with 127s.
  idx = 0;
  cv::dilate(block_img_, dilated_block_img_, cv::Mat(), cv::Point(-1, -1), 2);
  for(int r = 0; r < block_img_.rows; ++r) {
    for(int c = 0; c < block_img_.cols; ++c) {
      if(block_img_(r, c) == 0 && dilated_block_img_(r, c) == 255) {
        int y = r * height_step_ + height_step_ / 2;
        int x = c * width_step_ + width_step_ / 2;
        bg_fringe_markers->push_back(y * width_ + x);
        for(int y2 = r * height_step_; y2 < (r+1) * height_step_; ++y2)
          for(int x2 = c * width_step_; x2 < (c+1) * width_step_; ++x2)
            indices->push_back(y2 * width_ + x2);
      }
    }
  }
}

double BackgroundModel::transform(double x) const
{
  return weights_.coeffRef(0) * x * x + weights_.coeffRef(1) * x + weights_.coeffRef(2);
}

double BackgroundModel::transformDerivative(double x) const
{
  return 2 * weights_.coeffRef(0) * x + weights_.coeffRef(1);
}

double BackgroundModel::inverseTransform(double x) const
{
  double a = weights_(0);
  double b = weights_(1);
  double c = weights_(2) - x;

  ROS_ASSERT(b*b - 4*a*c >= 0);
  double val = sqrt(b*b - 4*a*c);
  double result0 = (-b + val) / (2*a);
  double result1 = (-b - val) / (2*a);

  if(min_depth_ - 1e-6 < result0 && result0 < max_depth_ + 1e-6) {
    ROS_ASSERT(!(min_depth_ - 1e-6 < result1 && result1 < max_depth_ + 1e-6));
    return result0;
  }
  else {
    ROS_ASSERT(min_depth_ - 1e-6 < result1 && result1 < max_depth_ + 1e-6);
    ROS_ASSERT(!(min_depth_ - 1e-6 < result0 && result0 < max_depth_ + 1e-6));
    return result1;
  }
}

void BackgroundModel::debug(int x, int y)
{
  if(width_step_ != 1 || height_step_ != 1) {
    ROS_WARN("Cannot do BackgroundModel::debug unless width_step_ and height_step_ are both 1.");
    return;
  }
  
  for(size_t i = 0; i < histograms_.size(); ++i)
    histograms_[i].debug_ = false;
  
  int idx = y * width_ + x;
  histograms_[idx].debug_ = true;
}
