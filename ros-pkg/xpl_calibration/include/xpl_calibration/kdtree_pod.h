#ifndef KDTREE_POD_H
#define KDTREE_POD_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pipeline/pod.h>
#include <rgbd_sequence/rgbd_sequence.h>

class KdTreePod : public pipeline::Pod
{
public:
  typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;
  
  DECLARE_POD(KdTreePod);
  KdTreePod(std::string name) :
    Pod(name),
    kdtree_(new KdTree())
  {
    declareInput<rgbd::Cloud::ConstPtr>("Cloud");
    declareOutput<KdTree::Ptr>("KdTree");
  }

  void compute()
  {
    rgbd::Cloud::ConstPtr cloud;
    pull("Cloud", &cloud);
    kdtree_->setInputCloud(cloud);
    push("KdTree", kdtree_);
  }

protected:
  KdTree::Ptr kdtree_;
};

#endif // KDTREE_POD_H
