#include <xpl_calibration/background_subtractor.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

void BackgroundSubtractor::compute()
{
  const vector<double>& min_distances = *pull<const vector<double>*>("MinDistances");
  const vector<double>& max_distances = *pull<const vector<double>*>("MaxDistances");
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  ROS_ASSERT(seq.size() > 0);
  ROS_ASSERT(seq.pcds_[0]->size() == min_distances.size());
  ROS_ASSERT(seq.pcds_[0]->size() == max_distances.size());
  
  fg_indices_.resize(seq.size());
  for(size_t i = 0; i < fg_indices_.size(); ++i)
    fg_indices_[i].clear();
  
  for(size_t i = 0; i < seq.size(); ++i)
    findForeground(*seq.pcds_[i], min_distances, max_distances, &fg_indices_[i]);

}

void BackgroundSubtractor::debug() const
{
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  for(size_t i = 0; i < seq.size(); ++i) {
    const Cloud& pcd = *seq.pcds_[i];
    const vector<int>& indices = fg_indices_[i];

    Cloud fg;
    fg.reserve(indices.size());
    for(size_t j = 0; j < indices.size(); ++j)
      fg.push_back(pcd[indices[j]]);

    ostringstream oss;
    oss << getDebugPath() << "-cloud" << setw(4) << setfill('0') << i << ".pcd";
    pcl::io::savePCDFileBinary(oss.str(), fg);
  }
}

void BackgroundSubtractor::findForeground(const Cloud& pcd,
					  const vector<double>& min_distances,
					  const vector<double>& max_distances,
					  vector<int>* indices) const
{
  indices->clear();
  for(size_t i = 0; i < pcd.size(); ++i) {
    double z = pcd[i].z;
    if(isinf(z))
      continue;
    ROS_ASSERT(min_distances[i] <= max_distances[i]);
    if(z < min_distances[i] || z > max_distances[i])
      indices->push_back(i);
  }
}

