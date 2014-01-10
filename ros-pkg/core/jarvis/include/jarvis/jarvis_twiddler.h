#ifndef JARVIS_TWIDDLER_H
#define JARVIS_TWIDDLER_H

#include <online_learning/evaluator.h>
#include <pipeline/twiddler.h>
#include <online_learning/grid_classifier.h>

class JarvisTwiddler : public pl::PipelineTwiddler
{
public:
  static const int MAX_MS_PER_OBJ = 5;
  //! One labeled dataset for each task.
  JarvisTwiddler(std::vector<TrackDataset> datasets, int num_threads);
  
  static bool isRequired(pl::Pod* pod);
  
protected:
  int num_threads_;
  std::vector<TrackDataset> datasets_;
  
  void improvementHook(const YAML::Node& config,
                       const YAML::Node& results,
                       std::string evalpath) const;
  YAML::Node evaluate(const YAML::Node& config, std::string evalpath);
  double objective(const YAML::Node& results) const;
  double runSingleEval(YAML::Node config, TrackDataset::ConstPtr train, TrackDataset::ConstPtr test) const;
  void splitDataset(const TrackDataset& td, double pct0,
                    TrackDataset* split0, TrackDataset* split1) const;
  double runMultipleEvals(std::string debugging_name, const TrackDataset& dataset,
                          YAML::Node config, int num_evals, double pct_train) const;
  
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
