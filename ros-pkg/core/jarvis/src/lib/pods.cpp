#include <map>
#include <vector>
#include <string>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <jarvis/pods.h>

using namespace std;
using namespace Eigen;
using namespace pl;

void BlobProjector::compute()
{
  Blob::Ptr blob = pull<Blob::Ptr>("Blob");
  if(!blob->cloud_)
    blob->project();
  push<Blob::ConstPtr>("ProjectedBlob", blob);
}

void BoundingBoxSize::compute()
{
  const Blob& blob = *pull<Blob::ConstPtr>("ProjectedBlob");
  const Cloud& cloud = *blob.cloud_;

  Vector4f minpt, maxpt;
  pcl::getMinMax3D(cloud, minpt, maxpt);
  size_ = (maxpt - minpt).head(3);

  push<const VectorXf*>("BoundingBoxSize", &size_);
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
    const vector<const pl::Outlet*>& outlets = it->second;
    for(size_t i = 0; i < outlets.size(); ++i) {
      Pod* upstream = outlets[i]->pod();
      string output_name = outlets[i]->name();
      dmap.addName(upstream->uniqueReadableId(output_name));
    }
  }

  return dmap;
}


/************************************************************
 * CloudOrienter
 ************************************************************/

void CloudOrienter::compute()
{
  const Blob& blob = *pull<Blob::ConstPtr>("ProjectedBlob");
  Cloud::ConstPtr cloud = blob.cloud_;

  pcl::PCA<Point> pca;
  pca.setInputCloud(cloud);
  pca.project(*cloud, *oriented_);

  push<Cloud::ConstPtr>("OrientedCloud", oriented_);
}

void CloudOrienter::debug() const
{
  const Blob& blob = *pull<Blob::ConstPtr>("ProjectedBlob");
  pcl::io::savePCDFileBinary(debugBasePath() + "-original.pcd", *blob.cloud_);
  pcl::io::savePCDFileBinary(debugBasePath() + "-oriented.pcd", *oriented_);
}

