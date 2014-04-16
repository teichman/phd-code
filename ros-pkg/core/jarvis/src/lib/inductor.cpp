#include <jarvis/inductor.h>
#include <jarvis/descriptor_pipeline.h>
#include <eigen_extensions/eigen_extensions.h>
#include <online_learning/clusterer.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

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

Inductor::Inductor(std::istream& in) :
  OnlineLearner(in)
{
  cout << "Deserialized.  unsupervised_: " << unsupervised_->size() << endl;
  deserialize(in);
}

void Inductor::entryHook(TrackDataset* td, const std::string& path) const
{
  cout << "Updating descriptors for data at path: \"" << path << "\"" << endl;
  double ms_per_obj = updateDescriptors(config_["Pipeline"], 24, td, up_);
  cout << "Done.  ms_per_obj: " << ms_per_obj << endl;

  ROS_ASSERT(nameMapping("dmap") == td->nameMapping("dmap"));
  td->applyNameMapping("cmap", nameMapping("cmap"));
  
  if(ms_per_obj > 0 && path != "") {
    cout << "[Inductor] Caching updated descriptors at \"" << path << "\"" << endl;
    td->save(path);
  }
}

void Inductor::chunkHook(TrackDataset* td) const
{
  cout << "Running Inductor::chunkHook." << endl;
  
  vector<Dataset::Ptr> tracks;
  tracks.reserve(td->size());

  // -- Tracks must be moving a bit for them to be used.
  //    This is because running group induction on stationary tracks 
  //    essentially reduces to self-learning - all the groups
  //    contain near-identical instances.
  for(size_t i = 0; i < td->size(); ++i) {
    Dataset::Ptr track = td->tracks_[i];
    if(track->empty())
      continue;
    // Blob::ConstPtr first_blob = boost::any_cast<Blob::ConstPtr>((*track)[0].raw());
    // Blob::ConstPtr last_blob = boost::any_cast<Blob::ConstPtr>((*track)[track->size() - 1].raw());
    // if(!first_blob->cloud_)
    //   first_blob->project(false);
    // if(!last_blob->cloud_)
    //   last_blob->project(false);

    // double dist = (last_blob->centroid_ - first_blob->centroid_).norm();
    // double dt = last_blob->sensor_timestamp_ - first_blob->sensor_timestamp_;
    // if(dist / dt > 0.05) {
    //   tracks.push_back(track);
    // }

    if(!isStatic(*track, *classifier_, 0.3, 3))
      tracks.push_back(track);
  }

  cout << "[Inductor::chunkHook]  Removing " << td->tracks_.size() - tracks.size() << " tracks because they did not move enough.  "
       << tracks.size() << " tracks remain." << endl;
  
  td->tracks_ = tracks;
}

void Inductor::retrospection(const TrackDataset& new_annotations, const std::vector<Label>& predictions)
{
  // -- De-induct all inducted examples that look similar to things we got wrong.
  Label unknown = VectorXf::Zero(nameMapping("cmap").size());
  vector<bool> deinduction_occurred(nameMapping("cmap").size(), false);
  for(size_t c = 0; c < nameMapping("cmap").size(); ++c) {
    int num_deinducted = 0;
    // For each annotated example that we got wrong
    for(size_t i = 0; i < new_annotations.size(); ++i) {
      Label annotation = new_annotations.label(i);
      if((predictions[i](c) > 0 && annotation(c) < 0) ||
         (predictions[i](c) < 0 && annotation(c) > 0))
      {
        // For each inducted example that has the same {-1, +1} label as the prediction,
        // de-induct if they look similar.
        int sign = predictions[i].sign()(c);
        for(size_t j = 0; j < unsupervised_->size(); ++j) {
          Label pred = unsupervised_->label(j);
          // New retrospection.  De-induct tracks that look roughly the same.
          if(pred.sign()(c) == sign &&
             similar(new_annotations[i], (*unsupervised_)[j], *classifier_, 0.7, 3))
          {
            if(pred.squaredNorm() > 1e-6)
              ++num_deinducted;
            (*unsupervised_)[j].setLabel(unknown);
            deinduction_occurred[c] = true;
          }
        }
      }
    }

    cout << "[Inductor::retrospection]  De-inducted " << num_deinducted << " objects of class " << nameMapping("cmap").toName(c) << endl;
  }

  // -- Reset classes for which de-induction occurred.
  scopeLockWrite;
  ROS_ASSERT(nameMappingsAreEqual(*classifier_));
  for(size_t c = 0; c < deinduction_occurred.size(); ++c) {
    if(deinduction_occurred[c]) {
      classifier_->setZero(c);
      cout << "[Inductor::retrospection]  Reset " << nameMapping("cmap").toName(c) << " classifier (index " << c << ")." << endl;
    }
  }
}

void Inductor::requestInductedSampleHook(TrackDataset* td, int cidx) const
{
  // -- Make a copy of the tracks and compute descriptors.
  TrackDataset cloned = *td->clone();
  entryHook(&cloned);
  
  // -- Sort the tracks according to confidence.  We want the most confident
  //    tracks to be sent while their less-confidently-classified near-duplicates
  //    get dropped.
  vector< pair<double, size_t> > index;
  index.reserve(cloned.size());
  for(size_t i = 0; i < cloned.size(); ++i) {
    Label pred = cloned.label(i);
    index.push_back(pair<double, size_t>(fabs(pred(cidx)), i));
  }
  sort(index.begin(), index.end(), greater< pair<double, size_t> >());  // descending

  // -- Filter out tracks that are near-duplicates.
  TrackDataset filtered;
  filtered.applyNameMappings(cloned);
  filtered.tracks_.reserve(cloned.size());
  for(size_t i = 0; i < index.size(); ++i) {
    size_t idx = index[i].second;
    bool unique = true;
    for(size_t j = 0; unique && j < filtered.size(); ++j)
      if(similar(cloned[idx], filtered[j], *classifier_, 0.9, 0))
        unique = false;
    if(unique)
      filtered.tracks_.push_back(Dataset::Ptr(new Dataset(cloned[idx])));
  }
  
  // -- Re-sort the tracks according to prediction absolute value
  //    for easy browsing.
  index.clear();
  for(size_t i = 0; i < filtered.size(); ++i) {
    Label pred = filtered.label(i);
    index.push_back(pair<double, size_t>(pred(cidx), i));
  }
  sort(index.begin(), index.end());  // ascending
  
  vector<Dataset::Ptr> tracks;
  tracks.reserve(filtered.size());
  for(size_t i = 0; i < index.size(); ++i)
    tracks.push_back(filtered.tracks_[index[i].second]);
  filtered.tracks_ = tracks;
  
  *td = filtered;
}

void Inductor::serialize(std::ostream& out) const
{
  OnlineLearner::serialize(out);
  eigen_extensions::serialize(up_, out);
  serializeYAML(config_, out);
}

void Inductor::deserialize(std::istream& in)
{
  //OnlineLearner::deserialize(in);  // This is done by the deserialization ctor.
  // I'm not sure I like this architecture.
  // It might be better to allow default ctors and uninitialized state, then
  // have init() and resume() functions.
  
  eigen_extensions::deserialize(in, &up_);
  deserializeYAML(in, &config_);
}

