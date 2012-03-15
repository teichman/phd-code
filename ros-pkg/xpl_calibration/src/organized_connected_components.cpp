#include <xpl_calibration/plane_finder.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

OrganizedConnectedComponents::OrganizedConnectedComponents(size_t min_inliers,
							   double distance_thresh) :
  min_inliers_(min_inliers),
  distance_thresh_(distance_thresh)
{
}

void OrganizedConnectedComponents::compute(const Cloud& pcd)
{
  ScopedTimer st("OrganizedConnectedComponents::compute");
  ROS_ASSERT(pcd.isOrganized());
  ROS_ASSERT(UNTOUCHED == -2); // enum isn't guaranteed to work with negative numbers.

  assignments_.clear();
  assignments_.resize(pcd.size(), UNTOUCHED);
  
  vector<bool> marked(pcd.size(), false);
  IndicesPtr unused(new vector<int>(pcd.size()));
  int num_components = 0;
  for(size_t i = 0; i < pcd.size(); ++i) {
    ROS_ASSERT(assignments_[i] != PROCESSING);
    if(isnan(pcd[i].x))
      assignments_[i] = NODATA;
    else if(assignments_[i] == UNTOUCHED) { 
      bool found = findComponent(pcd, i, num_components);
      if(found)
	++num_components;
    }
  }

  indices_.clear();
  indices_.resize(num_components);
  for(size_t i = 0; i < indices_.size(); ++i) {
    int num = 0;
    for(size_t j = 0; j < assignments_.size(); ++j)
      if(assignments_[j] == (int)i)
	++num;
    indices_[i].reserve(num);
  }
  
  for(size_t i = 0; i < assignments_.size(); ++i) {
    if(assignments_[i] >= 0)
      indices_[assignments_[i]].push_back(i);
  }


  for(size_t i = 0; i < indices_.size(); ++i)
    cout << "Object " << i << " has " << indices_[i].size() << " points." << endl;
}

bool OrganizedConnectedComponents::findComponent(const Cloud& pcd,
						 size_t center_idx,
						 int new_id)
{
  queue<size_t> que;
  que.push(center_idx);
  vector<size_t> inliers;
  inliers.push_back(center_idx);
  vector<int> indices;
  
  while(!que.empty()) {
    size_t idx = que.front();
    que.pop();
    indices.clear();
    getNeighbors(pcd, idx, &indices);

    for(size_t i = 0; i < indices.size(); ++i) {
      ROS_ASSERT(assignments_[indices[i]] == UNTOUCHED);
      inliers.push_back(indices[i]);
      que.push(indices[i]);
      assignments_[indices[i]] = PROCESSING;
    }
  }

  if(inliers.size() < min_inliers_) { 
    for(size_t i = 0; i < inliers.size(); ++i)
      assignments_[inliers[i]] = NONE;
    return false;
  }
  else {
    for(size_t i = 0; i < inliers.size(); ++i)
      assignments_[inliers[i]] = new_id;
    return true;
  }
}

void OrganizedConnectedComponents::getNeighbors(const Cloud& pcd,
						size_t center_idx,
						vector<int>* indices)
{
  int center_y = center_idx / pcd.width;
  int center_x = center_idx - center_y * pcd.width;
  ImageRegionIterator iri(cv::Size(pcd.width, pcd.height), 2);
  for(iri.setCenter(cv::Point2i(center_x, center_y)); !iri.done(); ++iri) {
    if(assignments_[iri.index()] != UNTOUCHED)
      continue;
    if(iri.index() == (int)center_idx)
      continue;

    double dist = pcl::euclideanDistance(pcd[iri.index()], pcd[center_idx]);
    if(dist < distance_thresh_)
      indices->push_back(iri.index());
  }
}
    
