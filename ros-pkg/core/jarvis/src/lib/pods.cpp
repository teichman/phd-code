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


/************************************************************
 * BlobProjector
 ************************************************************/

void BlobProjector::compute()
{
  Blob::Ptr blob = pull<Blob::Ptr>("Blob");
  if(!blob->cloud_)
    blob->project();

  push<Blob::ConstPtr>("ProjectedBlob", blob);
  push<Cloud::ConstPtr>("Cloud", blob->cloud_);
}

void BoundingBoxSize::compute()
{
  const Cloud& cloud = *pull<Cloud::ConstPtr>("Cloud");

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


/************************************************************
 * CentroidFinder
 ************************************************************/

void CentroidFinder::compute()
{
  const Cloud& cloud = *pull<Cloud::ConstPtr>("Cloud");
  pcl::compute3DCentroid(cloud, centroid_);
  descriptor_ = centroid_.head(3);
  push<const VectorXf*>("Centroid", &descriptor_);
}

void CentroidFinder::debug() const
{
  ofstream f((debugBasePath() + ".txt").c_str());
  f << "centroid_: " << centroid_.transpose() << endl;
  f << "descriptor_: " << descriptor_.transpose() << endl;
  f.close();
}


/************************************************************
 * NormalizedDensityHistogram
 ************************************************************/

void NormalizedDensityHistogram::compute()
{
  const Cloud& cloud = *pull<Cloud::ConstPtr>("Cloud");
  
  // -- Initialize bins if necessary.
  ROS_ASSERT(lower_limits_.size() == 3 && bins_.size() == 3);
  int num_bins = param<double>("NumBins");
  if(lower_limits_[0].rows() != num_bins) {
    for(size_t i = 0; i < lower_limits_.size(); ++i) {
      lower_limits_[i] = VectorXf::Zero(num_bins);
      bins_[i] = VectorXf::Zero(num_bins);
    }
  }

  // -- Compute min and max.
  Vector4f minpt, maxpt;
  pcl::getMinMax3D(cloud, minpt, maxpt);

  
  for(int i = 0; i < 3; ++i) {
    // Set lower limits of the bins.
    VectorXf& lower_limits = lower_limits_[i];
    VectorXf& bins = bins_[i];
    float minval = minpt(i);
    float maxval = maxpt(i);
    float binwidth = (maxval - minval) / num_bins;
    for(int j = 0; j < lower_limits.rows(); ++j)
      lower_limits(j) = minval + binwidth * j;
    
    // Fill bins with counts.
    for(size_t j = 0; j < cloud.size(); ++j) {
      float val = cloud[j].getVector3fMap().coeffRef(i);
      int idx = max<int>(0, min<int>(num_bins - 1, (val - minval) / binwidth));
      ++bins.coeffRef(idx);
    }

    // Normalize to sum to one.
    bins /= bins.sum();
    push<const VectorXf*>(names_[i], &bins);
  }
}

void NormalizedDensityHistogram::debug() const
{
  ofstream f((debugBasePath() + ".txt").c_str());
  for(size_t i = 0; i < lower_limits_.size(); ++i) {
    f << names_[i] << endl;
    f << "  lower_limits_: " << lower_limits_[i].transpose() << endl;
    f << "  bins_: " << bins_[i].transpose() << endl;
  }
  f.close();
}

