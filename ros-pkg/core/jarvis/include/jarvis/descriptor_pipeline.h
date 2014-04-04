#ifndef DESCRIPTOR_PIPELINE_H
#define DESCRIPTOR_PIPELINE_H

#include <online_learning/dataset.h>
#include <jarvis/pods.h>

class DescriptorPipeline
{
public:
  typedef boost::shared_ptr<DescriptorPipeline> Ptr;
  
  pl::Pipeline pl_;

  DescriptorPipeline();
  void initializeWithDefault();
  void initialize(YAML::Node plspec);
  static std::string defaultSpecificationPath();
  static YAML::Node defaultSpecification();
  static void registerPodTypes();
  const std::vector<const Eigen::VectorXf*>* computeDescriptors(Blob::ConstPtr blob);
  std::string reportTiming() const { return pl_.reportTiming(); }
  // It turns out this is really slow.  Use wisely.
  NameMapping dmap() const { return pl_.pod<DescriptorAggregator>()->dmap(); }
  void setDebug(bool debug) { pl_.setDebug(debug); }
  void setUpVector(const Eigen::VectorXf up);
  void reset() { pl_.reset(); }
};

double updateDescriptors(YAML::Node plspec, int num_threads, TrackDataset* td,
                         Eigen::VectorXf up = Eigen::VectorXf(), bool debug = false);
TrackDataset::Ptr loadDatasets(const std::vector<std::string> paths,
                               YAML::Node config,
                               const NameMapping& cmap = NameMapping(),
                               const Eigen::VectorXf& up = Eigen::VectorXf(),
                               bool verbose = false);
TrackDataset::Ptr loadDatasets(const std::vector<std::string> paths,
                               YAML::Node config,
                               const NameMapping& cmap = NameMapping(),
                               bool verbose = false);
  
#endif // DESCRIPTOR_PIPELINE_H
