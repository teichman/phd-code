#ifndef JARVIS_PODS_H
#define JARVIS_PODS_H

#include <pipeline/pipeline.h>
#include <name_mapping/name_mapping.h>
#include <jarvis/tracker.h>

class BlobProjector : public pl::Pod
{
public:
  DECLARE_POD(BlobProjector);
  BlobProjector(std::string name) :
    Pod(name)
  {
    declareInput<Blob::Ptr>("Blob");
    declareOutput<Blob::ConstPtr>("ProjectedBlob");
    declareOutput<Cloud::ConstPtr>("Cloud");
  }

  void compute();
};

class BoundingBoxSize : public pl::Pod
{
public:
  DECLARE_POD(BoundingBoxSize);
  BoundingBoxSize(std::string name) :
    Pod(name),
    size_(Eigen::VectorXf::Zero(3))
  {
    declareInput<Cloud::ConstPtr>("Cloud");
    declareOutput<const Eigen::VectorXf*>("BoundingBoxSize");
  }

  void compute();

protected:
  Eigen::VectorXf size_;
};

class DescriptorAggregator : public pl::Pod
{
public:
  DECLARE_POD(DescriptorAggregator);
  DescriptorAggregator(std::string name) :
    Pod(name)
  {
    declareMultiInput<const Eigen::VectorXf*>("Descriptors");
    declareOutput<const std::vector<const Eigen::VectorXf*>* >("AggregatedDescriptors");
    declareOutput<const NameMapping*>("DMap");
  }

  NameMapping dmap() const;
  
protected:
  std::vector<const Eigen::VectorXf*> aggregated_;
  NameMapping dmap_;
  
  void compute();
  void debug() const;
};

class CloudOrienter : public pl::Pod
{
public:
  DECLARE_POD(CloudOrienter);
  CloudOrienter(std::string name) :
    Pod(name),
    oriented_(new Cloud)
  {
    declareInput<Blob::ConstPtr>("ProjectedBlob");
    declareOutput<Cloud::ConstPtr>("OrientedCloud");
  }

  void compute();
  void debug() const;

protected:
  Cloud::Ptr oriented_;
};

class CloudSelector : public pl::Pod
{
public:
  DECLARE_POD(CloudSelector);
  CloudSelector(std::string name) :
    Pod(name)
  {
    declareInput<Blob::ConstPtr>("ProjectedBlob");
    declareOutput<Cloud::ConstPtr>("Cloud");
  }

  void compute() {
    Cloud::ConstPtr cloud = pull<Blob::ConstPtr>("ProjectedBlob")->cloud_;
    push("Cloud", cloud);
  }
};

class CentroidFinder : public pl::Pod
{
public:
  DECLARE_POD(CentroidFinder);
  CentroidFinder(std::string name) :
    Pod(name)
  {
    declareInput<Cloud::ConstPtr>("Cloud");
    declareOutput<const Eigen::VectorXf*>("Centroid");
  }

  void compute();
  void debug() const;

protected:
  Eigen::Vector4f centroid_;
  Eigen::VectorXf descriptor_;
};

#endif // JARVIS_PODS_H
