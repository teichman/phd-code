#include <map>
#include <vector>
#include <pipeline/outlet.h>
#include <ros/package.h>
#include <pcl/common/common.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace Eigen;

DescriptorPipeline::DescriptorPipeline() :
  pl_(1)
{
  registerPodTypes();
}

void DescriptorPipeline::registerPodTypes()
{
  REGISTER_POD_TEMPLATE(EntryPoint, Blob::ConstPtr);
  REGISTER_POD(BlobProjector);
  REGISTER_POD(BoundingBoxSize);
  REGISTER_POD(DescriptorAggregator);
}

YAML::Node DescriptorPipeline::defaultSpecification()
{
  string path = ros::package::getPath("jarvis") + "/config/default_descriptor_pipeline.yml";
  return YAML::LoadFile(path)["Pipeline"];
}

void DescriptorPipeline::initializeWithDefault()
{
  pl_.deYAMLize(defaultSpecification());
}

void BlobProjector::compute()
{
  Blob::Ptr blob = pull<Blob::Ptr>("Blob");
  if(!blob->cloud_)
    blob->project();
  push("ProjectedBlob", blob);
}

void BoundingBoxSize::compute()
{
  const Blob& blob = *pull<Blob::ConstPtr>("ProjectedBlob");
  const Cloud& cloud = *blob.cloud_;

  Vector4f minpt, maxpt;
  pcl::getMinMax3D(cloud, minpt, maxpt);
  size_ = (maxpt - minpt).head(3);

  push("BoundingBoxSize", &size_);
}



/************************************************************
 * DescriptorAggregator
 ************************************************************/

void DescriptorAggregator::compute()
{
  multiPull("Descriptors", &aggregated_);
  push<const std::vector<const VectorXf*>* >("AggregatedDescriptors", &aggregated_);

  if(dmap_.size() == 0)
    dmap_ = dmap();

  push<const NameMapping*>("DMap", &dmap_);
}

void DescriptorAggregator::debug() const
{
  ofstream f((debugBasePath() + ".txt").c_str());

  vector<string> names = upstreamOutputNames("Descriptors");
  
  int total = 0;
  for(size_t i = 0; i < aggregated_.size(); ++i) {
    f << "Descriptor " << i << " (" << names[i] << "): ";
    if(aggregated_[i]) {
      f << "present";
      total += aggregated_[i]->rows();
    }
    else
      f << "missing";
    f << endl;
  }

  f << endl;
  f.close();
}

NameMapping DescriptorAggregator::dmap() const
{
  NameMapping dmap;
  map<string, vector<const pl::Outlet*> >::const_iterator it;
  for(it = inputPipes().begin(); it != inputPipes().end(); ++it) {
    const vector<const Outlet*>& outlets = it->second;
    for(size_t i = 0; i < outlets.size(); ++i) {
      Pod* upstream = outlets[i]->pod();
      string output_name = outlets[i]->name();
      dmap.addName(upstream->uniqueReadableId(output_name));
    }
  }

  return dmap;
}
