#ifndef JARVIS_TWIDDLER_H
#define JARVIS_TWIDDLER_H

#include <pipeline/twiddler.h>
#include <online_learning/grid_classifier.h>

class JarvisTwiddler : public pl::PipelineTwiddler
{
public:
  static const int MAX_MS_PER_OBJ = 1;
  
  JarvisTwiddler(TrackDataset::Ptr train,
                 TrackDataset::Ptr test,
                 int num_threads);
  
  static bool isRequired(pl::Pod* pod);
  
protected:
  int num_threads_;
  TrackDataset::Ptr train_;
  TrackDataset::Ptr test_;
  TrackDataset::Ptr speed_check_;
  
  void improvementHook(const YAML::Node& config,
                       const YAML::Node& results,
                       std::string evalpath) const;
  YAML::Node evaluate(const YAML::Node& config, std::string evalpath);
  double objective(const YAML::Node& results) const;

  // -- Actions
  void twiddleNumCells(YAML::Node config) const;
  void twiddleTrainerThreshold(YAML::Node config) const;
  void deleteRandomPod(YAML::Node config) const;
  void addRawNormalizedHistogramBranch(YAML::Node config) const;
  void addOrientedNormalizedHistogramBranch(YAML::Node config) const;
};


#endif // JARVIS_TWIDDLER_H
