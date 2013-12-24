#include <jarvis/inductor.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace Eigen;

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
  double ms_per_obj = updateDescriptors(config_["Pipeline"], 24, td, up_);
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

  // -- Tracks must be moving a bit for them to be used.
  //    This is because running group induction on stationary tracks 
  //    essentially reduces to self-learning - all the groups
  //    contain near-identical instances.
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
    double dt = last_blob->sensor_timestamp_ - first_blob->sensor_timestamp_;
    if(dist / dt > 0.05) {
      tracks.push_back(track);
      cda.push_back(chunk_diagnostic_annotations->at(i));
    }
  }

  cout << "[Inductor::chunkHook]  Removing " << td->tracks_.size() - tracks.size() << " tracks because they did not move enough.  "
       << tracks.size() << " tracks remain." << endl;
  
  td->tracks_ = tracks;
  *chunk_diagnostic_annotations = cda;
}

bool similar(const Dataset& annotation, const Dataset& inducted)
{
  const NameMapping& dmap = annotation.nameMapping("dmap");
  //string name = "OrientedBoundingBoxSize.BoundingBoxSize:15595929600647926249";  // BoundingBoxSize from CloudOrienter.
  string name = "OrientedBoundingBoxSize.BoundingBoxSize:9048624352072648104";  // BoundingBoxSize from GravitationalCloudOrienter.
  if(!dmap.hasName(name)) {
    cout << dmap << endl;
    ROS_ASSERT(dmap.hasName(name));
  }
  size_t id = dmap.toId(name);

  double min_dist = numeric_limits<double>::max();
  int num_samples = 10;
  for(int i = 0; i < num_samples; ++i) {
    VectorXf* ann = annotation[rand() % annotation.size()].descriptors_[id];
    ROS_ASSERT(ann);
    ROS_ASSERT(ann->rows() == 3);
    for(int j = 0; j < num_samples; ++j) {
      VectorXf* ind = inducted[rand() % inducted.size()].descriptors_[id];
      ROS_ASSERT(ind);
      ROS_ASSERT(ind->rows() == 3);

      double max_pct_change = 0;
      for(int k = 0; k < ind->rows(); ++k) {
        // double pct_change = max(fabs(ann->coeffRef(k) - ind->coeffRef(k)) / ann->coeffRef(k),
        //                         fabs(ann->coeffRef(k) - ind->coeffRef(k)) / ind->coeffRef(k));
        if(ind->coeffRef(k) == 0)
          continue;
        double pct_change = fabs(ann->coeffRef(k) - ind->coeffRef(k)) / ind->coeffRef(k);
        max_pct_change = max(max_pct_change, pct_change);
      }
      
      min_dist = min(min_dist, max_pct_change);
    }
  }
  return (min_dist < 0.2);
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
          if(pred.sign()(c) == sign && fabs(pred(c)) <= fabs(predictions[i](c)) && similar(new_annotations[i], (*unsupervised_)[j])) {
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
  // scopeLockWrite;
  // ROS_ASSERT(nameMappingsAreEqual(*classifier_));
  // for(size_t c = 0; c < deinduction_occurred.size(); ++c) {
  //   if(deinduction_occurred[c]) {
  //     classifier_->setZero(c);
  //     cout << "[Inductor::retrospection]  Reset " << nameMapping("cmap").toName(c) << " classifier (index " << c << ")." << endl;
  //   }
  // }
}
