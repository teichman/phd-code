#ifndef JARVIS_TWIDDLER_H
#define JARVIS_TWIDDLER_H

#include <online_learning/evaluator.h>
#include <pipeline/twiddler.h>
#include <online_learning/grid_classifier.h>

class JarvisTwiddler : public pl::PipelineTwiddler
{
public:
  static const int MAX_MS_PER_OBJ = 5;
  
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
  void addHogBranch(YAML::Node config) const;
  void replaceHogBranch(YAML::Node config) const;
};

//! Assumes that config has a Pipeline node and a GlobalParams node at the root level.
//! GlobalParams should have NumCells and ObjThresh as members.
void evaluateConfig(YAML::Node config, int num_threads,
                    TrackDataset::Ptr train, TrackDataset::Ptr test,
                    GridClassifier::Ptr* gcp,
                    Evaluator::Ptr* evp);


#endif // JARVIS_TWIDDLER_H
