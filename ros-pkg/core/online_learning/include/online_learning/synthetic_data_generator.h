#ifndef SYNTHETIC_DATA_GENERATOR_H
#define SYNTHETIC_DATA_GENERATOR_H

#include <eigen_extensions/random.h>
#include <online_learning/dataset.h>

class ClassGenerator
{
public:
  //! means_[cluster][descriptor_space][element]
  std::vector< std::vector<Eigen::VectorXf> > means_;
  eigen_extensions::GaussianSampler gs_;
  NameMapping cmap_;
  NameMapping dmap_;
  uint32_t seed_;
  int cid_;
  double missing_probability_;
  DescriptorDimensionality dim_;

  //! Instance is assumed to be empty.
  void sample(Instance* instance);
  ClassGenerator(const DescriptorDimensionality& dimensionality, double descriptor_variance,
                 double mean_variance, double missing_probability,
                 uint32_t seed, int cid, const NameMapping& cmap, const NameMapping& dmap);
};

class SyntheticDataGenerator
{
public:
  std::vector<ClassGenerator> classes_;
  NameMapping cmap_;
  NameMapping dmap_;
  DescriptorDimensionality dim_;
    
  SyntheticDataGenerator(const DescriptorDimensionality& dimensionality,
			 double descriptor_variance, double mean_variance,
			 double missing_probability,
			 const NameMapping& cmap, const NameMapping& dmap);
  //! -2 => choose a class at random.
  Dataset::Ptr sampleDataset(size_t num_instances, int cid = -2);
  TrackDataset::Ptr sampleTrackDataset(size_t num_tracks, size_t num_instances_per_track);
  TrackDataset::Ptr sampleBackground(size_t num_tracks, size_t num_instances_per_track);
};


DescriptorDimensionality getDefaultDimensionality(size_t num_descriptors);
SyntheticDataGenerator getDefaultGenerator(int num_descriptors = 3);

inline NameMapping defaultClassMap()
{
  NameMapping cmap;
  cmap.addName("car");
  cmap.addName("pedestrian");
  cmap.addName("bicyclist");
  return cmap;
}

#endif // SYNTHETIC_DATA_GENERATOR_H
