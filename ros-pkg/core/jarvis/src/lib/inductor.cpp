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
  cout << "Updating descriptors for data at path: \"" << path << "\"" << endl;
  double ms_per_obj = updateDescriptors(config_["Pipeline"], 24, td);
  cout << "Done.  ms_per_obj: " << ms_per_obj << endl;

  ROS_ASSERT(nameMapping("dmap") == td->nameMapping("dmap"));
  td->applyNameMapping("cmap", nameMapping("cmap"));
  
  if(ms_per_obj > 0 && path != "") {
    cout << "[Inductor] Caching updated descriptors at \"" << path << "\"" << endl;
    td->save(path);
  }
}


void Inductor::chunkHook(TrackDataset* td, std::vector<Label>* chunk_diagnostic_annotations) const
{
  ROS_ASSERT(td->size() == chunk_diagnostic_annotations->size());
  vector<Dataset::Ptr> tracks;
  vector<Label> cda;
  tracks.reserve(td->size());
  cda.reserve(chunk_diagnostic_annotations->size());
  
  for(size_t i = 0; i < td->size(); ++i) {
    Dataset::Ptr track = td->tracks_[i];
    if(track->empty())
      continue;
    Blob::Ptr first_blob = boost::any_cast<Blob::Ptr>((*track)[0].raw_);
    Blob::Ptr last_blob = boost::any_cast<Blob::Ptr>((*track)[track->size() - 1].raw_);
    if(!first_blob->cloud_)
      first_blob->project(false);
    if(!last_blob->cloud_)
      last_blob->project(false);

    double dist = (last_blob->centroid_ - first_blob->centroid_).norm();
    if(dist > 0.03) {
      tracks.push_back(track);
      cda.push_back(chunk_diagnostic_annotations->at(i));
    }
  }

  cout << "[Inductor::chunkHook]  Removing " << td->tracks_.size() - tracks.size() << " tracks because they did not move enough.  "
       << tracks.size() << " tracks remain." << endl;
  
  td->tracks_ = tracks;
  *chunk_diagnostic_annotations = cda;
}
