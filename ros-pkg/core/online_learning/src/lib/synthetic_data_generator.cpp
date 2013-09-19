#include <online_learning/synthetic_data_generator.h>

using namespace std;
using namespace Eigen;
using namespace eigen_extensions;

ClassGenerator::ClassGenerator(const DescriptorDimensionality& dim,
                               double descriptor_variance, double mean_variance,
                               double missing_probability,
                               uint32_t seed, int cid,
                               const NameMapping& cmap, const NameMapping& dmap) :
  gs_(0, descriptor_variance, seed),
  cmap_(cmap),
  dmap_(dmap),
  seed_(seed),
  cid_(cid),
  missing_probability_(missing_probability),
  dim_(dim)
{
  ROS_ASSERT(dim_.size() == dmap_.size());
  means_.resize(50);
  for(size_t i = 0; i < means_.size(); ++i) { 
    means_[i].resize(dim_.size());
    GaussianSampler gs(0, mean_variance, seed_);
    for(size_t j = 0; j < dim_.size(); ++j) {
      means_[i][j].resize(dim_[j]);
      gs.sample(&means_[i][j]);
      //cout << "New cluster center: " << means_[i][j].transpose() << endl;
    }
  }
}

void ClassGenerator::sample(Instance* instance)
{
  instance->applyNameTranslator("cmap", NameTranslator(NameMapping(), cmap_));
  instance->applyNameTranslator("dmap", NameTranslator(NameMapping(), dmap_));

  instance->label_ = VectorXf::Ones(cmap_.size()) * -1;
  instance->label_(cid_) = 1;
  
  int cluster = rand() % means_.size();
  for(size_t i = 0; i < instance->descriptors_.size(); ++i) {
    if(((double)rand() / (double)RAND_MAX) < missing_probability_) {
      if(instance->descriptors_[i]) delete instance->descriptors_[i];
      instance->descriptors_[i] = NULL;
    }
    else { 
      if(!instance->descriptors_[i])
	instance->descriptors_[i] = new VectorXf(means_[cluster][i].rows());
      else
	instance->descriptors_[i]->resize(means_[cluster][i].rows());
      
      gs_.sample(instance->descriptors_[i]);
      *instance->descriptors_[i] += means_[cluster][i];
    }
  }
}

SyntheticDataGenerator::SyntheticDataGenerator(const DescriptorDimensionality& dim,
                                               double descriptor_variance, double mean_variance,
					       double missing_probability, const NameMapping& cmap,
                                               const NameMapping& dmap) :
  cmap_(cmap),
  dmap_(dmap),
  dim_(dim)
{
  for(size_t i = 0; i < cmap.size(); ++i) {
    classes_.push_back(ClassGenerator(dim_, descriptor_variance, mean_variance,
				      missing_probability, rand(), i, cmap, dmap));
  }
}

  
Dataset::Ptr SyntheticDataGenerator::sampleDataset(size_t num_instances, int cid)
{
  Dataset::Ptr dataset(new Dataset);
  dataset->applyNameMapping("cmap", cmap_);
  dataset->applyNameMapping("dmap", dmap_);
  
  dataset->instances_.resize(num_instances);
  for(size_t i = 0; i < dataset->instances_.size(); ++i) {
    int c = cid;
    if(cid == -2)
      c = rand() % classes_.size();
    classes_[c].sample(&dataset->instances_[i]);
  }

  return dataset;
}

TrackDataset::Ptr SyntheticDataGenerator::sampleTrackDataset(size_t num_tracks, size_t num_instances_per_track)
{
  TrackDataset::Ptr td(new TrackDataset);
  td->applyNameMapping("cmap", cmap_);
  td->applyNameMapping("dmap", dmap_);
  td->tracks_.resize(num_tracks);

  // Whatever, be slow.
  for(size_t i = 0; i < td->tracks_.size(); ++i) {
    //int cid = rand() % classes_.size();
    int cid = i % classes_.size();
    td->tracks_[i] = sampleDataset(num_instances_per_track, cid);

    // if(!getenv("NO_NOISE") && (double)rand() / RAND_MAX > 0.95) {
    //   Label rlab = VectorXf::Zero(cmap_.size());
    //   for(int j = 0; j < rlab.rows(); ++j) {
    //     if(rand() % 2)
    //       rlab(j) = 1;
    //     else
    //       rlab(j) = -1;
    //   }
    //   td->tracks_[i].setLabel(rlab);
    // }
  }

  return td;
}
  
DescriptorDimensionality getDefaultDimensionality(size_t num_descriptors)
{
  DescriptorDimensionality dim(num_descriptors);
  for(size_t i = 0; i < dim.size(); ++i)
    dim[i] = i+1;

  return dim;
}

NameMapping getDefaultSyntheticClassMap()
{
  NameMapping class_map;
  class_map.addName("car");
  class_map.addName("pedestrian");
  class_map.addName("bicyclist");
  class_map.addName("other1");
  class_map.addName("other2");
  class_map.addName("other3");
  return class_map;
}

SyntheticDataGenerator getDefaultGenerator(int num_descriptors)
{
  SyntheticDataGenerator sdg(getDefaultDimensionality(num_descriptors), 0.5, 5, 0.2, getDefaultSyntheticClassMap(), getStubDescriptorMap(num_descriptors));
  return sdg;
}
