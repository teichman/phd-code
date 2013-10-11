#ifndef INDUCTOR_H
#define INDUCTOR_H

#include <yaml-cpp/yaml.h>
#include <online_learning/tbssl.h>

class Inductor : public OnlineLearner
{
public:
  //! config["Pipeline"] is the descriptor pipeline.
  //! The other values are ignored.  classifier and trainer
  //! should be initialized with their own parameters manually.
  Inductor(YAML::Node config,
           double emax,
           size_t buffer_size,
           size_t max_track_length,
           GridClassifier::Ptr classifier,
           GridClassifier::BoostingTrainer::Ptr trainer,
           int max_iters,
           int snapshot_every,
           int evaluate_every,
           std::string output_dir,
           std::string unlabeled_dir,
           std::string saved_annotations_dir = "");

protected:
  YAML::Node config_;
  
  //! Updates descriptors and saves them to disk if necessary.
  void entryHook(TrackDataset* td, const std::string& path) const;
};

#endif // INDUCTOR_H
