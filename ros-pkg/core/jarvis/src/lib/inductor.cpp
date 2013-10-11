#include <jarvis/inductor.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;

Inductor::Inductor(YAML::Node config,
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
                   std::string saved_annotations_dir) :
  OnlineLearner(emax, buffer_size, max_track_length, classifier, trainer, max_iters, snapshot_every, evaluate_every, output_dir, unlabeled_dir, saved_annotations_dir),
  config_(config)
{
}

void Inductor::entryHook(TrackDataset* td, const std::string& path) const
{
  double ms_per_obj = updateDescriptors(config_["Pipeline"], 24, td);
  if(ms_per_obj > 0 && path != "") {
    cout << "[Inductor] Caching updated descriptors at \"" << path << "\"" << endl;
    td->save(path);
  }
}


