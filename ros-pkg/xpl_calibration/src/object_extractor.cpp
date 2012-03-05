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
  fg->reserve(fg_indices.size());
  for(size_t i = 0; i < fg_indices.size(); ++i)
    fg->push_back(pcd[fg_indices[i]]);
  
  // -- Run euclidean cluster extraction.
  HighResTimer hrt("Cluster extraction");
  hrt.start();
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(fg);
  cout << "Searching for objects among " << fg->size() << " foreground points." << endl;
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(param<double>("ClusterTolerance"));
  ec.setMinClusterSize(param<int>("MinClusterSize"));
  ec.setMaxClusterSize(1e6);
  ec.setSearchMethod(tree);
  ec.setInputCloud(fg);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  hrt.stop();
  cout << hrt.report() << endl;
  cout << "Found " << cluster_indices.size() << " clusters." << endl;

  // -- Add each cluster as an object.
  object_indices->resize(cluster_indices.size());
  for(size_t i = 0; i < cluster_indices.size(); ++i) {
    vector<int>& ind = cluster_indices[i].indices;
    Cloud::Ptr obj(new Cloud);
    obj->reserve(ind.size());
    object_indices->at(i).reserve(ind.size());
    for(size_t j = 0; j < ind.size(); ++j) { 
      obj->push_back(fg->at(ind[j]));
      object_indices->at(i).push_back(ind[j]);
    }

    objects->push_back(obj);
  }
}

void ObjectExtractor::debug() const
{
  cout << *this << endl;
  
  const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  const vector< vector<int> >& fg_indices = *pull<const vector< vector<int> >*>("ForegroundIndices");
  
  for(size_t i = 0; i < seq.size(); ++i) {
    cout << "Frame " << i << ": " << object_indices_[i].size() << " objects." << endl;
    Cloud vis = *seq.pcds_[i];
    for(size_t j = 0; j < object_indices_[i].size(); ++j) {
      cout << "  Object " << j << ": " << object_indices_[i][j].size() << " points." << endl;
      double r = (rand() % 256);
      double g = (rand() % 256);
      double b = (rand() % 256);
      for(size_t k = 0; k < object_indices_[i][j].size(); ++k) {
	int idx = fg_indices[i][object_indices_[i][j][k]];
	vis[idx].r = r;
	vis[idx].g = g;
	vis[idx].b = b;
      }
    }

    ostringstream oss;
    oss << getDebugPath() << "-cloud" << setw(4) << setfill('0') << i << ".pcd";
    pcl::io::savePCDFileBinary(oss.str(), vis);
  }
}
					      
