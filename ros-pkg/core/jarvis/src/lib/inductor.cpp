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
  return;  // Turning off the motion requirement below.
  
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

Eigen::ArrayXf computeNormalizedCellHistogram(const Dataset& track, size_t descriptor_id, const GridClassifier& gc)
{
  ROS_ASSERT(gc.numResolutions() == 1);  // Only deal with a single grid resolution for now.  This could be extended later.   
  vector<Grid*> grids = gc.grids_[0][descriptor_id];  // One grid per element of this descriptor space.
  ROS_ASSERT(!grids.empty());
  int num_cells_per_element = grids[0]->cells_.cols();  // Each element gets the same number of grid cells.
  int num_cells = grids.size() * num_cells_per_element;
  ArrayXf hist = ArrayXf::Zero(num_cells);
  for(size_t i = 0; i < grids.size(); ++i) {
    ROS_ASSERT(grids[i]);
    const Grid& grid = *grids[i];
    ROS_ASSERT(i == 0 || grid.cells_.cols() == grids[i-1]->cells_.cols());

    for(size_t j = 0; j < track.size(); ++j) {
      const Instance& frame = track[j];
      if(!frame[descriptor_id])
        continue;

      const VectorXf& descriptor = *frame[descriptor_id];
      size_t idx = grid.getCellIdx(descriptor.coeffRef(i));
      ROS_ASSERT(i * num_cells_per_element + idx < (size_t)hist.rows());
      ++hist(i * num_cells_per_element + idx);
    }
  }

  hist /= hist.sum();
  return hist;
}

float histogramIntersection(const Eigen::ArrayXf& hist0, const Eigen::ArrayXf& hist1)
{
  ROS_ASSERT(hist0.rows() == hist1.rows());

  float val = 0;
  for(int i = 0; i < hist0.rows(); ++i)
    val += min(hist0.coeffRef(i), hist1.coeffRef(i));
  return val;
}

//! annotation and inducted have descriptors computed.
bool similar(const Dataset& track0, const Dataset& track1, const GridClassifier& gc, double intersection_threshold, int max_different_dspaces)
{
  ROS_ASSERT(track0.nameMappingsAreEqual(track1));
  const NameMapping& dmap = track0.nameMapping("dmap");

  vector<string> dspaces;
  // -- less-gravity
  dspaces.push_back("OrientedBoundingBoxSize.BoundingBoxSize:14462707047095316535");
  dspaces.push_back("CloudOrienter.Eigenvalues:12988381052486413110");
  dspaces.push_back("CloudOrienter.RelativeCurvature:1286978089874286107");
  dspaces.push_back("HSVHistogram.Hue:14694502210542588030");
  dspaces.push_back("HSVHistogram.Saturation:10273388249095023270");
  dspaces.push_back("HSVHistogram.Value:8985795375221662105");
  // maybe include these too...
  // dspaces.push_back("SimpleTrajectoryStatistics05.Speed:6152030001663489947");
  // dspaces.push_back("SimpleTrajectoryStatistics10.Speed:9221731694321102499");
  // dspaces.push_back("EdginessEstimator.Edginess:6941314615084190538");
    
  // -- old
  // dspaces.push_back("OrientedBoundingBoxSize.BoundingBoxSize:9048624352072648104");  // BoundingBoxSize from GravitationalCloudOrienter.
  // dspaces.push_back("CloudOrienter.Eigenvalues:12466250795116632929");
  // dspaces.push_back("CloudOrienter.RelativeCurvature:11309880616745749126");
  // dspaces.push_back("HSVHistogram.Hue:14694502210542588030");
  // dspaces.push_back("HSVHistogram.Saturation:10273388249095023270");
  // dspaces.push_back("HSVHistogram.Value:8985795375221662105");

  int num_different_dspaces = 0;
  for(size_t i = 0; i < dspaces.size(); ++i) {
    if(!dmap.hasName(dspaces[i])) {
      cout << dmap << endl;
      ROS_ASSERT(dmap.hasName(dspaces[i]));
    }

    size_t id = dmap.toId(dspaces[i]);
    ArrayXf hist0 = computeNormalizedCellHistogram(track0, id, gc);
    ArrayXf hist1 = computeNormalizedCellHistogram(track1, id, gc);

    if(histogramIntersection(hist0, hist1) < intersection_threshold) {
      ++num_different_dspaces;
      if(num_different_dspaces > max_different_dspaces)
        return false;
    }
  }
  return true;
}

bool similar(const Dataset& annotation, const Dataset& inducted)
{
  const NameMapping& dmap = annotation.nameMapping("dmap");

  string name = "OrientedBoundingBoxSize.BoundingBoxSize:14462707047095316535";  // BoundingBoxSize from CloudOrienter.
  //string name = "OrientedBoundingBoxSize.BoundingBoxSize:9048624352072648104";  // BoundingBoxSize from GravitationalCloudOrienter.
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
          // New retrospection.
          if(pred.sign()(c) == sign &&
             similar(new_annotations[i], (*unsupervised_)[j], *classifier_, 0.9, 1))
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

