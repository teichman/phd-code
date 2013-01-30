#include <xpl_calibration/object_extractor.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

void ObjectExtractor::compute()
{
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  const vector<cv::Mat1b>& fg_imgs = *pull<const vector<cv::Mat1b>*>("ForegroundImages");
  const vector< vector<int> >& fg_indices = *pull<const vector< vector<int> >*>("ForegroundIndices");
  ROS_ASSERT(seq.size() == fg_imgs.size());
  ROS_ASSERT(seq.size() == fg_indices.size());

  objects_.clear();
  objects_.resize(seq.size());
  object_indices_.clear();
  object_indices_.resize(seq.size());
  
  for(size_t i = 0; i < fg_indices.size(); ++i)
    extractObjectsFromFrame(*seq.pcds_[i], fg_indices[i], &objects_[i], &object_indices_[i]);

  push<const Objects*>("Objects", &objects_);
  push<const ObjectIndices*>("ObjectIndices", &object_indices_);
}

void ObjectExtractor::extractObjectsFromFrame(const rgbd::Cloud& pcd,
					      const std::vector<int>& fg_indices,
					      std::vector<rgbd::Cloud::ConstPtr>* objects,
					      std::vector< std::vector<int> >* object_indices) const
{
  ROS_ASSERT(object_indices->empty());
  ROS_ASSERT(objects->empty());
  if(fg_indices.empty())
    return;
  
  // -- Get the cloud of just foreground points.
  Cloud::Ptr fg(new Cloud);
  *fg = pcd;
  for(size_t i = 0; i < fg->size(); ++i) { 
    fg->at(i).x = numeric_limits<double>::infinity();
    fg->at(i).y = numeric_limits<double>::infinity();
    fg->at(i).z = numeric_limits<double>::infinity();
  }
  for(size_t i = 0; i < fg_indices.size(); ++i)
    fg->at(fg_indices[i]) = pcd[fg_indices[i]];

  // -- Compute connected components.
  OrganizedConnectedComponents occ(param<int>("MinClusterPoints"),
				   param<double>("ClusterTolerance"));
  occ.compute(*fg);
  
  // -- Add each cluster as an object.
  //    Check if it is big enough.
  double min_sz = param<double>("MinClusterSize");
  object_indices->reserve(occ.indices_.size());
  objects->reserve(occ.indices_.size());
  for(size_t i = 0; i < occ.indices_.size(); ++i) {
    const vector<int>& ind = occ.indices_[i];
    Cloud::Ptr obj(new Cloud);
    obj->header.stamp = pcd.header.stamp;
    obj->reserve(ind.size());
    double min_x = numeric_limits<double>::max();
    double max_x = -numeric_limits<double>::max();
    double min_y = numeric_limits<double>::max();
    double max_y = -numeric_limits<double>::max();
    double min_z = numeric_limits<double>::max();
    double max_z = -numeric_limits<double>::max();
    for(size_t j = 0; j < ind.size(); ++j) { 
      Point pt = fg->at(ind[j]);
      obj->push_back(pt);
      if(pt.x < min_x)
	min_x = pt.x;
      if(pt.x > max_x)
	max_x = pt.x;
      if(pt.y < min_y)
	min_y = pt.y;
      if(pt.y > max_y)
	max_y = pt.y;
      if(pt.z < min_z)
	min_z = pt.z;
      if(pt.z > max_z)
	max_z = pt.z;
    }

    if(max_x - min_x > min_sz ||
       max_y - min_y > min_sz ||
       max_z - min_z > min_sz) { 
      object_indices->push_back(ind);
      objects->push_back(obj);
    }
  }
}

void ObjectExtractor::debug() const
{
  cout << *this << endl;
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  
  for(size_t i = 0; i < seq.size(); ++i) {
    if(i % 25)
      continue;
    
    cout << "Frame " << i << ": " << object_indices_[i].size() << " objects." << endl;
    Cloud vis = *seq.pcds_[i];
    for(size_t j = 0; j < object_indices_[i].size(); ++j) {
      cout << "  Object " << j << ": " << object_indices_[i][j].size() << " points." << endl;
      double r = (rand() % 256);
      double g = (rand() % 256);
      double b = (rand() % 256);
      for(size_t k = 0; k < object_indices_[i][j].size(); ++k) {
	ROS_ASSERT(object_indices_[i][j][k] < (int)vis.size());
	ROS_ASSERT(object_indices_[i][j][k] >= 0);
	
	int idx = object_indices_[i][j][k];
	vis[idx].r = r;
	vis[idx].g = g;
	vis[idx].b = b;
      }
    }

    ostringstream oss;
    oss << debugBasePath() << "-cloud" << setw(4) << setfill('0') << i << ".pcd";
    pcl::io::savePCDFileBinary(oss.str(), vis);
  }
}
					      
