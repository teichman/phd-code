#ifndef INDUCTOR_H
#define INDUCTOR_H

#include <online_learning/tbssl.h>
#include <yaml-cpp/yaml.h>

class Inductor : public OnlineLearner
{
public:
  typedef boost::shared_ptr<Inductor> Ptr;
  typedef boost::shared_ptr<const Inductor> ConstPtr;
  
  //! This needs to be set if you are using a config with a GravitationalCloudOrienter.
  Eigen::VectorXf up_;
  
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

  Inductor(std::istream& in);

protected:
  YAML::Node config_;
  
  //! Updates descriptors and saves them to disk if necessary.
  void entryHook(TrackDataset* td, const std::string& path = "") const;
  //! Removes tracks that don't include enough motion.
  void chunkHook(TrackDataset* td) const;
  void retrospection(const TrackDataset& new_annotations, const std::vector<Label>& predictions);
  void requestInductedSampleHook(TrackDataset* td, int cidx) const;
  
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  void snapshot();
};

// For retrospection, intersection_threshold = 0.7 and max_different_dspaces = 1 seemed
// pretty reasonable by eye.
bool similar(const Dataset& track0, const Dataset& track1, const GridClassifier& gc,
             double intersection_threshold, int max_different_dspaces);

#endif // INDUCTOR_H
