#include <xpl_calibration/gaussian_background_modeler.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

void GaussianBackgroundModeler::compute()
{
  // -- Initialize.
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  ROS_ASSERT(seq.pcds_[0]->isOrganized());
  size_t num_pixels = seq.pcds_[0]->size();
  means_.clear();
  stdevs_.clear();
  min_distances_.clear();
  max_distances_.clear();
  counts_.clear();
  
  means_.resize(num_pixels, 0);
  stdevs_.resize(num_pixels, 0);
  counts_.resize(num_pixels, 0);
  min_distances_.resize(num_pixels, 0);
  max_distances_.resize(num_pixels, 0);
  
  // -- Compute the means for all pixels.
  for(size_t i = 0; i < seq.size(); ++i) {
    const Cloud& pcd = *seq.pcds_[i];
    for(size_t j = 0; j < pcd.size(); ++j) {
      if(!pcl_isfinite(pcd[j].z))
        continue;
      means_[j] += pcd[j].z;
      ++counts_[j];
    }
  }
  for(size_t i = 0; i < means_.size(); ++i)
    means_[i] /= counts_[i];

  // -- Compute the stdevs for all pixels.
  for(size_t i = 0; i < seq.size(); ++i) {
    const Cloud& pcd = *seq.pcds_[i];
    for(size_t j = 0; j < pcd.size(); ++j) {
      if(!pcl_isfinite(pcd[j].z))
        continue;
      stdevs_[j] += pow(pcd[j].z - means_[j], 2);
    }
  }
  for(size_t i = 0; i < stdevs_.size(); ++i) { 
    stdevs_[i] = sqrt(stdevs_[i] / counts_[i]);
  }
  
  // -- Compute the min and max distances.
  for(size_t i = 0; i < means_.size(); ++i) {
    min_distances_[i] = means_[i] - stdevs_[i] * param<double>("Stdevs");
    max_distances_[i] = means_[i] + stdevs_[i] * param<double>("Stdevs");
    //cout << "mean: " << means_[i] << ", stdev: " << stdevs_[i] << ", min: " << min_distances_[i] << ", max: " << max_distances_[i] << endl;
  }
  
  push<const vector<double>*>("MinDistances", &min_distances_);
  push<const vector<double>*>("MaxDistances", &max_distances_);
}

void GaussianBackgroundModeler::debug() const
{
  cv::imwrite(getDebugPath() + "-maxdistance.png", visualizeVector(max_distances_));
  cv::imwrite(getDebugPath() + "-mindistance.png", visualizeVector(min_distances_));
  cv::imwrite(getDebugPath() + "-means.png", visualizeVector(means_));
  cv::imwrite(getDebugPath() + "-stdevs.png", visualizeVector(stdevs_));
  cv::imwrite(getDebugPath() + "-counts.png", visualizeVector(counts_));
  
  double mean_width = 0;
  for(size_t i = 0; i < max_distances_.size(); ++i) {
    ROS_ASSERT(min_distances_[i] <= max_distances_[i]);
    mean_width += max_distances_[i] - min_distances_[i];
  }
  mean_width /= (double)max_distances_.size();
  cout << "Mean width: " << mean_width << endl;
}

cv::Mat1f GaussianBackgroundModeler::getZBuffer(const rgbd::Cloud& pcd) const
{
  cv::Mat1f zbuf(cv::Size(pcd.width, pcd.height), -1);
  for(int y = 0; y < zbuf.rows; ++y) {
    for(int x = 0; x < zbuf.cols; ++x) {
      int idx = y * zbuf.cols + x;
      if(!isnan(pcd[idx].z))
        zbuf(y, x) = pcd[idx].z;
    }
  }

  return zbuf;
}

cv::Mat1b GaussianBackgroundModeler::visualizeVector(const std::vector<double>& vec) const
{
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  const Cloud& pcd = *seq.pcds_[0];
  cv::Mat1f vis(cv::Size(pcd.width, pcd.height), 0);
  
  double max = -numeric_limits<double>::max();
  for(int y = 0; y < vis.rows; ++y) {
    for(int x = 0; x < vis.cols; ++x) {
      int idx = y * vis.cols + x;
      vis(y, x) = vec[idx];
      if(vec[idx] > max)
        max = vec[idx];
    }
  }
  for(int y = 0; y < vis.rows; ++y)
    for(int x = 0; x < vis.cols; ++x)
      vis(y, x) /= max;
  cv::Mat1b vis1b(vis.size(), 0);
  for(int y = 0; y < vis.rows; ++y)
    for(int x = 0; x < vis.cols; ++x)
      vis1b(y, x) = vis(y, x) * 255;

  return vis1b;
}
