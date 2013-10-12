#ifndef DESCRIPTOR_PIPELINE_H
#define DESCRIPTOR_PIPELINE_H

#include <online_learning/dataset.h>
#include <jarvis/pods.h>

class DescriptorPipeline
{
public:
  pl::Pipeline pl_;

  DescriptorPipeline();
  void initializeWithDefault();
  void initialize(YAML::Node plspec);
  static std::string defaultSpecificationPath();
  static YAML::Node defaultSpecification();
  static void registerPodTypes();
  const std::vector<const Eigen::VectorXf*>* computeDescriptors(Blob::Ptr blob);
  std::string reportTiming() const { return pl_.reportTiming(); }
  NameMapping dmap() const { return pl_.pod<DescriptorAggregator>()->dmap(); }
  void setDebug(bool debug) { pl_.setDebug(debug); }
};

double updateDescriptors(YAML::Node plspec, int num_threads, TrackDataset* td, bool debug = false);
TrackDataset::Ptr loadDatasets(YAML::Node config, const std::vector<std::string> paths);
  
#endif // DESCRIPTOR_PIPELINE_H
