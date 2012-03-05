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
  
  for(size_t i = 0; i < fg_indices.size(); ++i)
    extractObjectsFromFrame(*seq.pcds_[i], fg_indices[i], &objects_[i]);

  push<const Objects*>("Objects", &objects_);
}

void ObjectExtractor::extractObjectsFromFrame(const rgbd::Cloud& pcd,
					      const std::vector<int>& indices,
					      std::vector<rgbd::Cloud::ConstPtr>* objects) const
{

  // -- Get the cloud of just foreground points.
  Cloud::Ptr fg(new Cloud);
  fg->resize(indices.size());
  for(size_t i = 0; i < indices.size(); ++i)
    fg->push_back(pcd[indices[i]]);
    
  // -- Run euclidean cluster extraction.
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(fg);
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(param<double>("ClusterTolerance"));
  ec.setMinClusterSize(param<int>("MinClusterSize"));
  ec.setMaxClusterSize(1e6);
  ec.setSearchMethod(tree);
  ec.setInputCloud(fg);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  cout << "Found " << cluster_indices.size() << " clusters." << endl;

  // -- Add each cluster as an object.
  for(size_t i = 0; i < cluster_indices.size(); ++i) {
    vector<int>& ind = cluster_indices[i].indices;
    Cloud::Ptr obj(new Cloud);
    obj->reserve(ind.size());
    for(size_t j = 0; j < ind.size(); ++j)
      obj->push_back(fg->at(ind[j]));

    objects->push_back(obj);
  }
}
					      
