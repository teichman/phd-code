#include <online_learning/tbssl.h>
#include <glob.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

OnlineLearner::OnlineLearner(double emax,
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
  active_learning_(false),
  emax_(emax),
  buffer_size_(buffer_size),
  max_track_length_(max_track_length),
  classifier_(classifier),
  trainer_(trainer),
  annotated_(new TrackDataset),
  autobg_(new TrackDataset),
  unsupervised_(new TrackDataset),
  validation_(new TrackDataset),
  ann_counter_(0),
  viewable_unsupervised_(new TrackDataset),
  max_iters_(max_iters),
  snapshot_every_(snapshot_every),
  evaluate_every_(evaluate_every),
  output_dir_(output_dir),
  unlabeled_dir_(unlabeled_dir),
  saved_annotations_dir_(saved_annotations_dir),
  input_auto_annotations_dir_(output_dir + "/input_auto_annotations"),
  input_hand_annotations_dir_(output_dir + "/input_hand_annotations"),
  iter_(0),
  paused_(false)
{
  applyNameMappings(*classifier);
}

void OnlineLearner::copyClassifier(GridClassifier* classifier)
{
  scopeLockRead;
  *classifier = *classifier_;
}


inline std::vector<std::string> glob(const std::string& pat)
{
  glob_t glob_result;
  glob(pat.c_str(), GLOB_TILDE, NULL, &glob_result);
  vector<string> ret;
  for(unsigned int i = 0; i < glob_result.gl_pathc; ++i)
    ret.push_back(string(glob_result.gl_pathv[i]));
  globfree(&glob_result);
  return ret;
}

void OnlineLearner::updateViewableUnsupervised()
{
  boost::unique_lock<boost::shared_mutex> ulock(viewable_unsupervised_mutex_);

  ROS_ASSERT(unsupervised_->size() == unsupervised_logodds_.size());
  
  TrackDataset& uns = *unsupervised_;
  TrackDataset& vuns = *viewable_unsupervised_;
  vector<double>& hashes = viewable_unsupervised_hashes_;
    
  vuns.tracks_.clear();
  hashes.clear();
  if(uns.empty())
    return;

  // -- Copy over the relevant parts to viewable_unsupervised_.
  vuns.tracks_.resize(uns.size());
  for(size_t i = 0; i < vuns.tracks_.size(); ++i)
    vuns.tracks_[i] = Dataset::Ptr(new Dataset);
  vuns.applyNameMappings(uns);
  int nc = nameMapping("cmap").size();
  for(size_t i = 0; i < vuns.size(); ++i) {
    Dataset& vtrack = vuns[i];
    Dataset& utrack = uns[i];
    vtrack.instances_.resize(utrack.instances_.size());
    for(size_t j = 0; j < vtrack.size(); ++j) {
      // This should be a shared_ptr of some sort and
      // thus free to copy.
      vtrack[j].raw_ = utrack[j].raw_;  
      // If not inducted, set the label to zero.
      // Otherwise use the classifier track prediction.
      if((utrack[j].label_.array() == 0).all())
        vtrack[j].label_ = VectorXf::Zero(nc);
      else
        vtrack[j].label_ = unsupervised_logodds_[i];
    }
  }

  // -- Update the hashes.
  hashes.resize(vuns.size());
  ROS_ASSERT(vuns.size() == uns.size());
  for(size_t i = 0; i < hashes.size(); ++i) {
    ROS_ASSERT(i < uns.size());
    hashes[i] = uns[i].hash();
  }
}

void OnlineLearner::pushAnnotationForInducted(double hash, const Label& label)
{
  hand_mutex_.lock();
  vu_incoming_hashes_.push_back(hash);
  vu_incoming_labels_.push_back(label);
  hand_mutex_.unlock();
}

void OnlineLearner::annotateUnsupervised(double hash, const Label& label)
{
  TrackDataset& uns = *unsupervised_;
  TrackDataset::Ptr ann(new TrackDataset);
  ann->applyNameMappings(uns);
  int num = 0;
  for(size_t i = 0; i < uns.size(); ++i) {
    if(hash == uns[i].hash()) {
      uns[i].setLabel(label);
      // Make a deep copy of the track.  This is essential.
      // Otherwise, handleAnnotatedData calls inductDataset,
      // which in turn changes the labels on tracks in incoming_annotated_.
      // Total disaster.
      ann->tracks_.push_back(uns.copy(i));
      ++num;
    }
  }
  entryHook(ann.get());
  incoming_annotated_.push_back(ann);

  if(num == 0)
    ROS_WARN_STREAM("OnlineLearner::annotateUnsupervised failed to find a match for hash with label " << label.transpose() << std::flush);
  if(num > 1)
    ROS_WARN_STREAM("OnlineLearner::annotateUnsupervised found " << num << " matches for hash with label " << label.transpose() << std::flush);
}

void OnlineLearner::loadSavedAnnotations()
{
  if(saved_annotations_dir_ == "")
    return;

  ostringstream oss;
  oss << saved_annotations_dir_ << "/iter" << setw(5) << setfill('0') << iter_ << "/*.td";
  vector<string> paths = glob(oss.str());
  if(!paths.empty()) {
    cout << "Found saved annotations: " << endl;
    for(size_t i = 0; i < paths.size(); ++i) {
      cout << "  " << paths[i] << endl;
          
      TrackDataset::Ptr td = loadTrackDataset(paths[i]);
      ROS_ASSERT(annotated_->nameMappingsAreEqual(*td));
      pushHandLabeledDataset(td);
    }
  }
}

void OnlineLearner::handleAnnotatedData()
{

  boost::unique_lock<boost::shared_mutex> ulock(hand_mutex_);

  // -- Check for new annotations for the inducted dataset.
  for(size_t i = 0; i < vu_incoming_hashes_.size(); ++i) {
    annotateUnsupervised(vu_incoming_hashes_[i], vu_incoming_labels_[i]);
  }
  vu_incoming_hashes_.clear();
  vu_incoming_labels_.clear();

  // -- If there's nothing new, don't do anything.
  if(incoming_annotated_.empty())
    return;

  cout << "[OnlineLearner] incoming_annotated_.size() is " << incoming_annotated_.size() << endl;
  
  // -- Record stats.
  for(size_t i = 0; i < incoming_annotated_.size(); ++i) {
    stats_.incrementAnnotation(*incoming_annotated_[i]);
  }

  // -- Aggregate the new annotations.
  TrackDataset aggregated = *incoming_annotated_[0];  // Copy name mappings.
  ROS_ASSERT(nameMappingsAreEqual(aggregated));
  for(size_t i = 1; i < incoming_annotated_.size(); ++i)
    aggregated += *incoming_annotated_[i];
  incoming_annotated_.clear();
  saveByClassAndLabel(aggregated, iter_dir_ + "/annotated");
  
  // -- Split them between annotated_ and validation_.
  TrackDataset for_annotated;
  for_annotated.applyNameMappings(aggregated);
  for(size_t i = 0; i < aggregated.size(); ++i, ++ann_counter_) {
    // Disabled for now.
    if(false && ann_counter_ > 20 && ann_counter_ % 3 == 0) {
      validation_->tracks_.push_back(aggregated.tracks_[i]);
    }
    else {
      for_annotated.tracks_.push_back(aggregated.tracks_[i]);
    }
  }
  annotated_->tracks_.insert(annotated_->tracks_.end(),
                             for_annotated.tracks_.begin(), for_annotated.tracks_.end());  

  // -- Search for duplicate tracks in unsupervised_.  Remove them.
  // We actually don't want to use similar tracks.  This could have the
  // unintended side effect of de-inducting many tracks that are
  // correctly-labeled.  The hash is more appropriate.
  // TODO: Caching the hashes would be a good idea if this is slow.
  vector<double> uns_hashes(unsupervised_->size());
  for(size_t i = 0; i < unsupervised_->size(); ++i) {
    uns_hashes[i] = (*unsupervised_)[i].hash();
  }
  vector<bool> to_remove(unsupervised_->size(), false);
  for(size_t i = 0; i < for_annotated.size(); ++i) {
    double hash = for_annotated[i].hash();
    for(size_t j = 0; j < uns_hashes.size(); ++j)
      if(hash == uns_hashes[j])
        to_remove[j] = true;
  }
  vector<Dataset::Ptr> tracks;
  vector<Label> uls;
  tracks.reserve(buffer_size_);
  uls.reserve(buffer_size_);
  for(size_t i = 0; i < to_remove.size(); ++i) {
    if(!to_remove[i]) {
      tracks.push_back(unsupervised_->tracks_[i]);
      uls.push_back(unsupervised_logodds_[i]);
    }
  }
  size_t num_before = unsupervised_->size();
  unsupervised_->tracks_ = tracks;
  unsupervised_logodds_ = uls;
  cout << "Dropped " << num_before - unsupervised_->size() << " tracks in unsupervised_ that were identical to new annotations." << endl;
  
  // -- Classify the new annotated tracks that didn't end up in validation_.
  vector<Label> predictions(for_annotated.size());
  lockRead();
  ROS_ASSERT(classifier_->nameMappingsAreEqual(for_annotated));
#pragma omp parallel for
  for(size_t i = 0; i < for_annotated.size(); ++i)
    predictions[i] = classifier_->classifyTrack(for_annotated[i]);
  unlockRead();

  // -- De-induct tracks that the new annotated tracks indicate 
  //    should not have been inducted.
  retrospection(for_annotated, predictions);
}

void OnlineLearner::retrospection(const TrackDataset& new_annotations, const std::vector<Label>& predictions)
{
  // -- Estimate emin and emax.
  VectorXf emin = -VectorXf::Ones(nameMapping("cmap").size()) * emax_;
  VectorXf emax = VectorXf::Ones(nameMapping("cmap").size()) * emax_;
  for(size_t c = 0; c < nameMapping("cmap").size(); ++c) { 
    for(size_t i = 0; i < new_annotations.size(); ++i) {
      Label annotation = new_annotations.label(i);
      if(predictions[i](c) > 0 && annotation(c) < 0)
        emin(c) = min(emin(c), -predictions[i](c));
      if(predictions[i](c) < 0 && annotation(c) > 0)
        emax(c) = max(emax(c), -predictions[i](c));
    }
  }

  // -- Re-induct the unsupervised dataset using the noise estimates.
  ObjectiveIndex index;
  cout << "Estimated emin: " << emin.transpose() << endl;
  cout << "Estimated emax: " << emax.transpose() << endl;
  vector< vector<Label> > frame_logodds;
  inductDataset(emin, emax, unsupervised_.get(), &index,
                &unsupervised_logodds_, &frame_logodds);
  saveInductionAccuracy("retrospection");

  // -- Reset classes for which de-induction (may have) occurred.
  scopeLockWrite;
  ROS_ASSERT(nameMappingsAreEqual(*classifier_));
  for(size_t c = 0; c < nameMapping("cmap").size(); ++c) {
    if(emin(c) < -emax_ - 1e-3 || emax(c) > emax_ + 1e-3) {
      classifier_->setZero(c);
      cout << "Reset " << nameMapping("cmap").toName(c) << " classifier (index " << c << ")." << endl;
    }
  }
}

TrackDataset::Ptr OnlineLearner::getNextUnlabeledChunk()
{
  // -- Load the next unlabeled td.
  getNextPath(unlabeled_dir_, ".td", &td_path_);
  TrackDataset::Ptr unlabeled_chunk = loadTrackDataset(td_path_);
  // Split long tracks into many smaller tracks.
  // This helps keep the training sets balanced and makes it easier to induct tracks.
  splitTracksFixedLength(max_track_length_, unlabeled_chunk.get());
  cout << "Adding unlabeled_chunk with num tracks: " << unlabeled_chunk->size() << endl;
  
  // Remove duplicate tracks.  Otherwise, if you're looking at the same data over and over
  // again, you'll end up getting just a few of the same thing.
  removeDuplicates(unlabeled_chunk.get());
    
  // Strip labels, if any.
  Label unl = VectorXf::Zero(nameMapping("cmap").size());
  for(size_t i = 0; i < unlabeled_chunk->size(); ++i) {
    (*unlabeled_chunk)[i].setLabel(unl);
  }
  stats_.incrementUnlabeled(*unlabeled_chunk);

  return unlabeled_chunk;
}

void OnlineLearner::loadInputTDFiles()
{
  {
    vector<string> paths = glob(input_hand_annotations_dir_ + "/*.td");
    if(!paths.empty()) {
      cout << "Found new .td files: " << endl;
      for(size_t i = 0; i < paths.size(); ++i) {
        cout << "  " << paths[i] << endl;
        TrackDataset::Ptr td = loadTrackDataset(paths[i]);
        ROS_ASSERT(annotated_->nameMappingsAreEqual(*td));
        pushHandLabeledDataset(td);
        bfs::remove(paths[i]);
      }
    }
  }
  {
    vector<string> paths = glob(input_auto_annotations_dir_ + "/*.td");
    if(!paths.empty()) {
      cout << "Found new .td files: " << endl;
      for(size_t i = 0; i < paths.size(); ++i) {
        cout << "  " << paths[i] << endl;
        TrackDataset::Ptr td = loadTrackDataset(paths[i]);
        ROS_ASSERT(autobg_->nameMappingsAreEqual(*td));
        pushAutoLabeledDataset(td);
        bfs::remove(paths[i]);
      }
    }
  }
}

void OnlineLearner::removePerfectAndNonInducted(TrackDataset* unlabeled_chunk) const
                                                
{
  // -- Induct unlabeled_chunk.
  VectorXf emin = -VectorXf::Ones(nameMapping("cmap").size()) * emax_;
  VectorXf emax = VectorXf::Ones(nameMapping("cmap").size()) * emax_;
  cout << "Using emin = " << emin.transpose() << endl;
  cout << "Using emax = " << emax.transpose() << endl;
  ObjectiveIndex throwaway_index;
  vector<Label> track_logodds;
  vector< vector<Label> > frame_logodds;
  inductDataset(emin, emax, unlabeled_chunk, &throwaway_index, &track_logodds, &frame_logodds);

  // In the experiments on Junior, tracks almost never contained only frames
  // that were classified perfectly.  I thought maybe this part of the code
  // should be removed, but in the kitchen experiments it turns out we have
  // a significant number of perfectly-classified tracks.
  
  // -- Only accept tracks with errors we can learn from.
  int num_with_errors = 0;
  int num_perfect = 0;
  ROS_ASSERT(frame_logodds.size() == unlabeled_chunk->size());
  for(size_t i = 0; i < frame_logodds.size(); ++i) {
    Dataset& track = (*unlabeled_chunk)[i];
    ROS_ASSERT(frame_logodds[i].size() == track.size());
    Label ind = track.label().sign();  // {-1, 0, +1}^n

    // If this track wasn't inducted as anything, skip it.
    if((ind.array() == 0).all())
      continue;

    bool has_errors = false;
    for(size_t j = 0; j < track.size() && !has_errors; ++j) {
      Label pred = frame_logodds[i][j].sign();  // {-1, 0, +1}^n
      ROS_ASSERT(ind.rows() == pred.rows());
      for(int c = 0; c < ind.rows() && !has_errors; ++c) {
        if(ind(c) == 0)
          continue;
        if(pred(c) != ind(c))
          has_errors = true;
      }
    }
    if(!has_errors) {
      track.setLabel(VectorXf::Zero(ind.rows()));
      ++num_perfect;
    }
    else
      ++num_with_errors;
  }
  cout << "[OnlineLearner::deinductPerfect] total: " << num_with_errors + num_perfect << endl;
  cout << "[OnlineLearner::deinductPerfect] num_with_errors: " << num_with_errors << endl;
  cout << "[OnlineLearner::deinductPerfect] num_perfect: " << num_perfect << endl;

  // -- Remove anything that could not be inducted.  These won't be useful to us later,
  //    so removing them now will just speed things up.  Also, we don't want unlabeled tracks
  //    in unsupervised_ to be replaced by non-inductable unlabeled tracks in unlabeled_chunk.
  //    This is because the former are often tracks de-inducted by retrospection which are
  //    actually quite likely to be (correctly) inducted later.  If these are replaced by
  //    random unlabeled tracks from unlabeled_chunk, the recovery process after retrospection
  //    is likely to be much slower.

  vector<Dataset::Ptr> tracks;
  tracks.reserve(unlabeled_chunk->size());
  for(size_t i = 0; i < unlabeled_chunk->size(); ++i) {
    const Dataset& track = (*unlabeled_chunk)[i];
    if((track.label().array() == 0).all())
      continue;
    tracks.push_back(unlabeled_chunk->tracks_[i]);
  }
  // cout << "[OnlineLearner::deinductPerfect] Before pruning: " << endl;
  // cout << unlabeled_chunk->status("  ", false) << endl;
  unlabeled_chunk->tracks_ = tracks;
  // cout << "[OnlineLearner::deinductPerfect] After pruning: " << endl;
  // cout << unlabeled_chunk->status("  ", false) << endl;
}

void OnlineLearner::balance(const ObjectiveIndex& index)
{
  for(size_t c = 0; c < nameMapping("cmap").size(); ++c) {
    ArrayXf ann_counts = VectorXf::Zero(2);
    for(size_t i = 0; i < annotated_->size(); ++i) {
      if(annotated_->label(i).sign()(c) > 0)
        ++ann_counts(0);
      else if(annotated_->label(i).sign()(c) < 0)
        ++ann_counts(1);
    }
    if((ann_counts == 0).all())
      continue;

    ArrayXf ind_counts = VectorXf::Zero(2);
    for(size_t i = 0; i < unsupervised_->size(); ++i) {
      if(unsupervised_->label(i).sign()(c) > 0)
        ++ind_counts(0);
      else if(unsupervised_->label(i).sign()(c) < 0)
        ++ind_counts(1);
    }

    double mult = (ind_counts / ann_counts).minCoeff();
    ROS_ASSERT(!isnan(mult));
    ArrayXi desired = (mult * ann_counts).cast<int>();
    ROS_ASSERT((desired <= ind_counts.cast<int>()).all());

    // Make sure index is sorted in descending order.
    for(size_t i = 1; i < index.size(); ++i) 
      ROS_ASSERT(index[i].first <= index[i-1].first && index[i].first >= 0);
    
    // De-induct the least useful tracks so that we get back to a balanced inducted set.
    ArrayXi num = ArrayXi::Zero(2);
    for(size_t i = 0; i < index.size(); ++i) {
      size_t idx = index[i].second;
      Label pred = unsupervised_->label(idx);
      if(pred(c) > 0) {
        if(num(0) < desired(0))
          ++num(0);
        else {
          Label label = unsupervised_->tracks_[idx]->label();
          label(c) = 0;
          unsupervised_->tracks_[idx]->setLabel(label);
        }
      }
      else if(pred(c) < 0) {
        if(num(1) < desired(1))
          ++num(1);
        else {
          Label label = unsupervised_->tracks_[idx]->label();
          label(c) = 0;
          unsupervised_->tracks_[idx]->setLabel(label);
        }
      }
    }
  }
}

void OnlineLearner::inductionStep(TrackDataset* unlabeled_chunk)
{
  // -- Induct the unlabeled chunk and the unsupervised dataset.
  {
    ScopedTimer st("Adding chunk to unsupervised dataset");
    size_t expected_size = unsupervised_->size() + unlabeled_chunk->size();
    *unsupervised_ += *unlabeled_chunk;  // TrackDataset uses shared_ptrs so we don't need to free unlabeled_chunk.
    ROS_ASSERT(unsupervised_->size() == expected_size);
  }

  VectorXf emin = -VectorXf::Ones(nameMapping("cmap").size()) * emax_;
  VectorXf emax = VectorXf::Ones(nameMapping("cmap").size()) * emax_;
  cout << "Using emin = " << emin.transpose() << endl;
  cout << "Using emax = " << emax.transpose() << endl;
  ObjectiveIndex index;  // The double is \sum_f \sum_c loss.
  vector< vector<Label> > frame_logodds;
  inductDataset(emin, emax, unsupervised_.get(), &index, &unsupervised_logodds_, &frame_logodds);

  sort(index.begin(), index.end(), greater< pair<double, size_t> >());  // descending
  //balance(index);
  prune(index);
}

void OnlineLearner::prune(const ObjectiveIndex& index)
{
  // -- Initialize things.
  vector<Dataset::Ptr> tracks;
  vector<Label> uls;
  tracks.reserve(buffer_size_);
  uls.reserve(buffer_size_);

  int num_classes = nameMapping("cmap").size();
  ArrayXi pos_counts = ArrayXi::Zero(num_classes);
  ArrayXi neg_counts = ArrayXi::Zero(num_classes);
  int num_desired = buffer_size_ / (num_classes * 2);

  // Make sure index is sorted in descending order.
  for(size_t i = 1; i < index.size(); ++i) 
    ROS_ASSERT(index[i].first <= index[i-1].first && index[i].first >= 0);

  // -- Keep the most useful inducted tracks.
  for(size_t i = 0; i < index.size(); ++i) {
    size_t idx = index[i].second;
    Label pred = unsupervised_->label(idx);

    // Determine what class problem this instance belongs to.
    // Make sure we're not using mutual exclusion.
    int c;
    pred.array().abs().maxCoeff(&c);
    for(int j = 0; j < pred.rows(); ++j)
      ROS_ASSERT(j == c || pred(j) == 0);

    if(pred(c) > 0 && pos_counts(c) < num_desired) {
      ++pos_counts(c);
      tracks.push_back(unsupervised_->tracks_[idx]);
      uls.push_back(unsupervised_logodds_[idx]);
    }
    else if(pred(c) < 0 && neg_counts(c) < num_desired) { 
      ++neg_counts(c);
      tracks.push_back(unsupervised_->tracks_[idx]);
      uls.push_back(unsupervised_logodds_[idx]);
    }
  }

  // -- If there is space left over in the buffer, fill it with unlabeled tracks.
  for(size_t i = 0; i < index.size() && tracks.size() < buffer_size_; ++i) {
    size_t idx = index[i].second;
    Label pred = unsupervised_->label(idx);
    if((pred.sign().array() == 0).all()) {
      tracks.push_back(unsupervised_->tracks_[idx]);
      uls.push_back(unsupervised_logodds_[idx]);
    }
  }
      
  ROS_ASSERT(pos_counts.maxCoeff() <= num_desired);
  ROS_ASSERT(neg_counts.maxCoeff() <= num_desired);
  ROS_ASSERT(tracks.size() <= (size_t)buffer_size_);
  ROS_ASSERT(tracks.size() == uls.size());
  cout << "pos_counts: " << pos_counts.transpose() << endl;
  cout << "neg_counts: " << neg_counts.transpose() << endl;
  cout << "tracks.size(): " << tracks.size() << endl;

  // Deallocate the dropped tracks.
  unsupervised_->tracks_ = tracks;
  unsupervised_logodds_ = uls;
  cout << "Buffer size: " << unsupervised_->size() << endl;
}

void OnlineLearner::_run()
{  
  if(!bfs::exists(output_dir_))
    bfs::create_directory(output_dir_);
  if(!bfs::exists(input_auto_annotations_dir_))
    bfs::create_directory(input_auto_annotations_dir_);
  if(!bfs::exists(input_hand_annotations_dir_))
    bfs::create_directory(input_hand_annotations_dir_);

  while(true) {    
    // -- Check if we're done.
    if(bfs::exists(output_dir_ + "/stop"))
      break;
    lockRead();
    if(quitting_) {
      unlockRead();
      break;
    }
    unlockRead();

    // -- Check for saved annotated data.
    loadSavedAnnotations();
    loadInputTDFiles();
    
    // -- Set up timing and results directory.
    HighResTimer hrt("OnlineLearner iteration time");
    hrt.start();
    ROS_DEBUG_STREAM("OnlineLearner starting iter " << iter_ << flush);
    ostringstream oss;
    oss << output_dir_ << "/iter" << setw(5) << setfill('0') << iter_;
    iter_dir_ = oss.str();
    bfs::create_directory(iter_dir_);
    ofstream file;
    file.open((iter_dir_ + "/learner_status.txt").c_str());
    file << status("  ") << endl;
    file.close();

    // Update the data that is publicly viewable.
    updateViewableUnsupervised();
    
    /************************************************************
     * Training
     ************************************************************/
    
    // -- If paused, wait here until the user is done adding
    //    new annotations.
    int paused_iter = 0;
    while(true) {
      {
        boost::unique_lock<boost::shared_mutex> ulock(hand_mutex_);
        if(!paused_)
          break;
      }
      usleep(1e6);

      if(paused_iter % 60 == 0)
        cout << "OnlineLearner is paused." << endl;
      ++paused_iter;
    }

    // -- Process all new annotations.
    handleAnnotatedData();    

    // -- Check name mappings.
    lockRead();
    ROS_ASSERT(nameMappingsAreEqual(*classifier_));
    ROS_ASSERT(classifier_->nameMappingsAreEqual(*unsupervised_));
    ROS_ASSERT(classifier_->nameMappingsAreEqual(*annotated_));
    unlockRead();

    // -- Train the classifier on Du + Da.
    vector<TrackDataset::ConstPtr> datasets;
    datasets.push_back(annotated_);
    datasets.push_back(autobg_);
    if(!active_learning_)
      datasets.push_back(unsupervised_);
    vector<Indices> indices;
    for(size_t i = 0; i < datasets.size(); ++i)
      indices.push_back(Indices::All(datasets[i]->size()));
    
    {
      ScopedTimer st("OnlineLearner: Training");
      scopeLockWrite;
      trainer_->train(datasets, indices);
    }

    /************************************************************
     * Induction
     ************************************************************/
    TrackDataset::Ptr unlabeled_chunk = getNextUnlabeledChunk();
    //removePerfectAndNonInducted(unlabeled_chunk.get());
    chunkHook(unlabeled_chunk.get());
    inductionStep(unlabeled_chunk.get());

    /************************************************************
     * Bookkeeping
     ************************************************************/

    // -- Save things and evaluate if it's time.
    if(evaluate_every_ > 0 && iter_ % evaluate_every_ == 0) {
      evaluate();
      //saveInductionExamples();
      classifier_->save(iter_dir_ + "/classifier.gc");      
    }
    saveInductionAccuracy("induction");
    
    // Need to update timing information before saving the snapshot so that it will
    // be recorded.  This also has the side effect (bug, feature?) of not including
    // snapshot time in the total elapsed time.
    hrt.stop();
    ROS_DEBUG_STREAM("Completed iter " << iter_ << " in " << hrt.getMinutes() << " minutes.");
    stats_.elapsed_ += hrt.getSeconds();

    if(snapshot_every_ == 1)
      snapshot();
    else if(snapshot_every_ > 0 && iter_ % snapshot_every_ == 0 && iter_ != 0)
      snapshot();

    ++iter_;
    {
      scopeLockRead;
      if(iter_ == max_iters_)
        break;
    }
  }
}

void OnlineLearner::viewableUnsupervised(TrackDataset* viewable_unsupervised, vector<double>* hashes) const
{
  viewable_unsupervised_mutex_.lock();
  *viewable_unsupervised = *viewable_unsupervised_;
  *hashes = viewable_unsupervised_hashes_;
  viewable_unsupervised_mutex_.unlock();
}

void OnlineLearner::removeDuplicates(TrackDataset* td) const
{
  set<double> hashes;
  {
    ScopedTimer st("OnlineLearner::removeDuplicates - hashing unsupervised_");
    for(size_t i = 0; i < unsupervised_->size(); ++i) {
      double hash = (*unsupervised_)[i].hash();
      if(hashes.count(hash))
        ROS_WARN("OnlineLearner::removeDuplicates found a collision in unsupervised_.");
      hashes.insert(hash);
    }
  }
  
  vector<bool> valid(td->size(), true);
  {
    ScopedTimer st("OnlineLearner::removeDuplicates - checking for collisions");
    int num_collisions = 0;
    for(size_t i = 0; i < td->size(); ++i) {
      double hash = td->tracks_[i]->hash();
      if(hashes.count(hash)) {
        ++num_collisions;
        valid[i] = false;
      }
    }
    cout << "Found " << num_collisions << " collisions." << endl;
  }

  {
    ScopedTimer st("OnlineLearner::removeDuplicates - copying tracks");
    vector<Dataset::Ptr> tracks;
    tracks.reserve(td->size());
    for(size_t i = 0; i < valid.size(); ++i)
      if(valid[i])
        tracks.push_back(td->tracks_[i]);

    td->tracks_ = tracks;
  }
}

void OnlineLearner::saveInductionExamples() const
{
  ROS_FATAL("This function assumes single induction.");
  abort();
  
  TrackDataset uns = *unsupervised_;  // Copy the shared_ptrs.
  random_shuffle(uns.tracks_.begin(), uns.tracks_.end());  // Randomize the order.

  size_t num_desired = 100;
  for(size_t i = 0; i < nameMapping("cmap").size(); ++i) {
    TrackDataset td;
    td.applyNameMappings(*this);

    // -- Clone a sampling of the tracks.
    for(size_t j = 0; j < uns.size() && td.size() < num_desired; ++j) {
      const Label& label = uns[j].label();
      if(label(i) > 0)
        td.tracks_.push_back(uns.tracks_[j]->clone());
    }

    // -- Crop those tracks down to a reasonable length
    //    so we don't use up so much disk space.
    cropTracks(30, &td);

    // -- Drop all the descriptors.  We only want to look at the raw data
    //    for debugging, and we can save disk space this way.
    NameMapping dmap;
    dmap.addName("Placeholder");
    td.applyNameMapping("dmap", dmap);
    
    // -- Save.
    string path = iter_dir_ + "/example-" + nameMapping("cmap").toName(i) + ".td";
    cout << "[OnlineLearner] Saving examples to " << path << endl;
    td.save(path);
  }
}

void OnlineLearner::saveInductionAccuracy(const std::string& basename) const
{
  // -- Track stats.
  {
    VectorXi unk = VectorXi::Zero(nameMapping("cmap").size());
    VectorXi ind = VectorXi::Zero(nameMapping("cmap").size());
    VectorXi pos = VectorXi::Zero(nameMapping("cmap").size());
    VectorXi neg = VectorXi::Zero(nameMapping("cmap").size());
              
    for(size_t i = 0; i < unsupervised_->size(); ++i) {
      Label prediction = unsupervised_->label(i);
      for(size_t c = 0; c < nameMapping("cmap").size(); ++c) {
        if(prediction(c) != 0)
          ++ind(c);
        if(prediction(c) > 0)
          ++pos(c);
        else if(prediction(c) < 0)
          ++neg(c);
      }
    }

    string path = iter_dir_ + "/" + basename + "-track.txt";
    ofstream f;
    int spacing = 10;
    f.open(path.c_str());
    f << setw(20) << "CLASS"
      << setw(spacing) << "UNK" << setw(spacing) << "IND"
      << setw(spacing) << "POS" << setw(spacing) << "NEG"
      << setw(spacing) << "TOT"
      << endl;
    f << "------------------------------------------------------------------------------------------------------------------------" << endl;
    for(size_t c = 0; c < nameMapping("cmap").size(); ++c)
      f << setw(20) << nameMapping("cmap").toName(c)
        << setw(spacing) << unk(c) << setw(spacing) << ind(c)
        << setw(spacing) << pos(c) << setw(spacing) << neg(c)
        << setw(spacing) << unsupervised_->size()
        << endl;
    f.close();
  }

  // -- Frame stats.
  {
    VectorXi unk = VectorXi::Zero(nameMapping("cmap").size());
    VectorXi ind = VectorXi::Zero(nameMapping("cmap").size());
    VectorXi pos = VectorXi::Zero(nameMapping("cmap").size());
    VectorXi neg = VectorXi::Zero(nameMapping("cmap").size());
  
    for(size_t i = 0; i < unsupervised_->size(); ++i) {
      const Dataset& track = (*unsupervised_)[i];
      Label prediction = unsupervised_->label(i);
      for(size_t c = 0; c < nameMapping("cmap").size(); ++c) {
        if(prediction(c) != 0)
          ind(c) += track.size();
        if(prediction(c) > 0)
          pos(c) += track.size();
        else if(prediction(c) < 0)
          neg(c) += track.size();
      }
    }

    string path = iter_dir_ + "/" + basename + "-frame.txt";
    ofstream f;
    int spacing = 10;
    f.open(path.c_str());
    f << setw(20) << "CLASS"
      << setw(spacing) << "UNK" << setw(spacing) << "IND"
      << setw(spacing) << "POS" << setw(spacing) << "NEG"
      << setw(spacing) << "TOT"
      << endl;
    f << "------------------------------------------------------------------------------------------------------------------------" << endl;
    for(size_t c = 0; c < nameMapping("cmap").size(); ++c)
      f << setw(20) << nameMapping("cmap").toName(c)
        << setw(spacing) << unk(c) << setw(spacing) << ind(c)
        << setw(spacing) << pos(c) << setw(spacing) << neg(c)
        << setw(spacing) << unsupervised_->totalInstances()
        << endl;
    f.close();
  }
}


void OnlineLearner::evaluate()
{
  if(!test_) {
    ROS_DEBUG("No test set provided.  Not evaluating.");
    return;
  }
  
  lockRead();
  HighResTimer hrt("OnlineLearner: evaluating");
  hrt.start();
  ROS_DEBUG_STREAM("OnlineLearner: evaluating...");
  Evaluator ev(classifier_);
  MatrixXf annotations, predictions;
  ev.evaluateParallel(*test_, &annotations, &predictions);
  ev.plot_ = false;
  bfs::create_directory(iter_dir_ + "/test_results/");
  ev.saveResults(iter_dir_ + "/test_results/");
  hrt.stop();
  ROS_DEBUG_STREAM(hrt.report() << flush);
  eigen_extensions::save(annotations, iter_dir_ + "/test_track_annotations.eig");
  eigen_extensions::save(predictions, iter_dir_ + "/test_track_predictions.eig");
  nameMapping("cmap").save(iter_dir_ + "/cmap.txt");
  // -- Save evaluation results for the annotated data, too.
  Evaluator ev_ann(classifier_);
  ev_ann.plot_ = false;
  ev_ann.evaluateParallel(*annotated_);
  bfs::create_directory(iter_dir_ + "/annotation_results/");
  ev_ann.saveResults(iter_dir_ + "/annotation_results/");

  // -- ... and the validation_ set.
  Evaluator ev_val(classifier_);
  ev_val.plot_ = false;
  ev_val.evaluateParallel(*validation_);
  bfs::create_directory(iter_dir_ + "/validation_results/");
  ev_val.saveResults(iter_dir_ + "/validation_results/");

  unlockRead();
}

void OnlineLearner::snapshot()
{
  scopeLockRead;
  
  // -- Serialize OnlineLearner.
  ScopedTimer st("Serializing OnlineLearner");
  ROS_DEBUG_STREAM("Serializing OnlineLearner.");
  
  // -- Delete the old one from two snapshots ago, if it exists.
  ostringstream oss_old_learner_path;
  oss_old_learner_path << output_dir_ << "/learner.ol." << setw(5) << setfill('0') << (iter_ - 2 * snapshot_every_);
  string old_learner_path = oss_old_learner_path.str();
  if(bfs::exists(old_learner_path))
    bfs::remove(old_learner_path);
  
  // -- Save the current snapshot.
  ostringstream oss_learner_filename;
  oss_learner_filename << "learner.ol." << setw(5) << setfill('0') << iter_;
  string learner_filename = oss_learner_filename.str();
  string learner_path = output_dir_ + "/" + learner_filename;
  save(learner_path);
  
  // -- Update the symlink for the most recent learner.
  if(bfs::exists(output_dir_ + "/learner.ol"))
    bfs::remove(output_dir_ + "/learner.ol");
  bfs::create_symlink(learner_filename, output_dir_ + "/learner.ol");
}

void OnlineLearner::inductDataset(const Eigen::VectorXf& emin, const Eigen::VectorXf& emax,
                                  TrackDataset* td, ObjectiveIndex* aggregate_index,
                                  std::vector<Label>* logodds,
                                  std::vector< std::vector<Label> >* frame_logodds) const
{
  ROS_ASSERT(frame_logodds);
  ROS_ASSERT(classifier_->nameMappingsAreEqual(*td));
  ROS_ASSERT((size_t)emax.rows() == nameMapping("cmap").size());
  ROS_ASSERT((size_t)emin.rows() == nameMapping("cmap").size());
  ROS_ASSERT(emax.minCoeff() >= 0 && emin.maxCoeff() <= 0);
  int num_classes = nameMapping("cmap").size();
  
  ScopedTimer st("OnlineLearner::inductDataset");
  scopeLockRead;  // We'll be using the classifier.
  
  // -- Set up the aggregate index.  This will contain the objective function for each track
  //    after induction is done.
  aggregate_index->clear();
  aggregate_index->resize(td->size());
  for(size_t i = 0; i < td->size(); ++i)
    aggregate_index->at(i) = pair<double, size_t>(0, i);

  
  // -- Induct each track individually.
  //    Use a random order for better core utilization.
  vector<size_t> index(td->size());
  for(size_t i = 0; i < index.size(); ++i) 
    index[i] = i;
  random_shuffle(index.begin(), index.end());

  logodds->clear();
  logodds->resize(index.size());
  frame_logodds->clear();
  frame_logodds->resize(index.size());
                   
#pragma omp parallel for
  for(size_t i = 0; i < index.size(); ++i) {
    size_t track_idx = index[i];
    
    Dataset& track = (*td)[track_idx];
    if(track.size() == 0)
      continue;

    // -- Compute the frame predictions.
    vector<Label>& predictions = frame_logodds->at(i);
    predictions.clear();
    predictions.resize(track.size());
    for(size_t j = 0; j < track.size(); ++j) { 
      predictions[j] = classifier_->classify(track[j]);
      ROS_ASSERT(predictions[j].rows() == num_classes);
    }

    // -- Compute the overall track prediction and store for later use.
    Label track_prediction = VectorXf::Zero(num_classes);
    for(size_t j = 0; j < predictions.size(); ++j) { 
      track_prediction += predictions[j];
    }
    track_prediction /= (float)track.size();
    // Check that prior isn't messed up.  I haven't been using it for a while.
    ROS_ASSERT((classifier_->prior().array() == 0).all());
    // ... but I might use a prior later.
    track_prediction += classifier_->prior();
    logodds->at(track_idx) = track_prediction;

    // We're going to evaluate \sum_c \sum_f exp(-y^c H^c(x_f)) for all possible labelings.
    // Construct possible labelings here.
    vector<ArrayXd> possible_labels;  // "Unknown" is handled specially.  All other possibilities will go here.
    //possible_labels.push_back(ArrayXd::Ones(num_classes) * -1);  // This should maybe be a part of dual induction.
    for(int c = 0; c < num_classes; ++c) {
      // Single induction with mutual exclusion.
      // ArrayXd lab = ArrayXd::Ones(num_classes) * -1;
      // lab(c) = 1;
      // possible_labels.push_back(lab);

      // Single induction without mutual exclusion.
      // ArrayXd lab = ArrayXd::Zero(num_classes);
      // lab(c) = 1;
      // possible_labels.push_back(lab);

      // Dual induction without mutual exclusion.
      ArrayXd lab = ArrayXd::Zero(num_classes);
      lab(c) = 1;
      possible_labels.push_back(lab);
      lab(c) = -1;
      possible_labels.push_back(lab);
    }

    ArrayXd objectives = ArrayXd::Zero(possible_labels.size());  // The objective function term for each possible labeling will go here.
    double objective_none = track.size() * num_classes;  // "Unknown" is handled separately for efficiency.
    for(size_t j = 0; j < predictions.size(); ++j) {
      // Loop over all allowed labelings.  Add up objective for each.
      for(size_t k = 0; k < possible_labels.size(); ++k) {
        const ArrayXd& lab = possible_labels[k];
        // Construct worst-case error vector for the proposed labeling.
        // If the label for a class is zero, the error doesn't do anything.
        ArrayXd err = ArrayXd::Zero(num_classes);
        for(int c = 0; c < num_classes; ++c) {
          if(lab(c) > 0)
            err(c) = emin(c);
          else if(lab(c) < 0)
            err(c) = emax(c);
        }

        // Add to the objective value for each possible labeling.
        objectives(k) += (-lab * (predictions[j].array().cast<double>() + err)).exp().sum();
      }
      // Terminate early if we know we can't induct this track.
      if(objective_none <= (objectives.minCoeff() + 1e-6))
        break;
    }

    // -- Set the label based on the best objective function value.
    if(objective_none <= (objectives.minCoeff() + 1e-6)) {
      track.setLabel(VectorXf::Zero(num_classes));
      // Note this is not objective_none.
      // That's because this track has no value in the training phase.
      aggregate_index->at(track_idx).first = 0;  
    }
    else {
      int idx = -1;
      aggregate_index->at(track_idx).first = objectives.minCoeff(&idx);
      VectorXf label = possible_labels[idx].matrix().cast<float>();
      track.setLabel(label);
    }
  }
}

std::string OnlineLearner::status(const std::string& prefix) const
{
  boost::unique_lock<boost::shared_mutex> ulock(hand_mutex_);
  
  ostringstream oss;
  oss << prefix << "============================================================" << endl;
  oss << prefix << "= Algorithm params" << endl;
  oss << prefix << "============================================================" << endl;
  oss << prefix << "emax_: " << emax_ << endl;
  oss << prefix << "buffer_size_: " << buffer_size_ << endl;
  oss << prefix << "max_track_length_: " << max_track_length_ << endl;
  oss << prefix << "============================================================" << endl;
  oss << prefix << "= Algorithm data" << endl;
  oss << prefix << "============================================================" << endl;
  oss << prefix << "Test dataset: " << endl;
  if(test_)
    oss << test_->status(prefix + "  ", false);
  else
    oss << prefix << " None." << endl;
  oss << trainer_->status(prefix + "  ");
  oss << prefix << "------------------------------------------------------------" << endl;
  oss << prefix << "Incoming annotated datasets: " << incoming_annotated_.size() << endl;
  for(size_t i = 0; i < incoming_annotated_.size(); ++i) {
    oss << prefix << "  Incoming annotated dataset #" << i+1 << " / " << incoming_annotated_.size() << endl;
    oss << incoming_annotated_[i]->status(prefix + "    ", false);
  }
  oss << prefix << "------------------------------------------------------------" << endl;
  oss << prefix << "Hand-annotated training dataset:" << endl;
  oss << annotated_->status(prefix + "  ", false);
  oss << prefix << "------------------------------------------------------------" << endl;
  oss << prefix << "Auto-annotated training dataset:" << endl;
  oss << autobg_->status(prefix + "  ", false);
  oss << prefix << "------------------------------------------------------------" << endl;
  oss << prefix << "Unsupervised dataset:" << endl;
  oss << unsupervised_->status(prefix + "  ", false);
  oss << prefix << "------------------------------------------------------------" << endl;
  oss << prefix << "Validation dataset:" << endl;
  oss << validation_->status(prefix + "  ", false);

  oss << prefix << "============================================================" << endl;
  oss << prefix << "= Bookkeeping params" << endl;
  oss << prefix << "============================================================" << endl;
  oss << prefix << "  max_iters_: " << max_iters_ << endl;
  oss << prefix << "  snapshot_every_: " << snapshot_every_ << endl;
  oss << prefix << "  evaluate_every_: " << evaluate_every_ << endl;
  oss << prefix << "  output_dir_: " << output_dir_ << endl;
  oss << prefix << "  unlabeled_dir_: " << unlabeled_dir_ << endl;
  oss << prefix << "  saved_annotations_dir_: " << saved_annotations_dir_ << endl;
  
  oss << prefix << "============================================================" << endl;
  oss << prefix << "= Bookkeeping data" << endl;
  oss << prefix << "============================================================" << endl;
  oss << prefix << "  Iteration number: " << iter_ << endl;
  oss << prefix << "  Stats: " << endl;
  oss << prefix << "  td_path_: " << td_path_ << endl;
  oss << stats_.status(prefix + "  ");
    
  return oss.str();
}

void OnlineLearner::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  scopeLockWrite;
  hand_mutex_.lock();
  ROS_ASSERT(!running_);

  // -- Algorithm data
  ROS_WARN("OnlineLearner::_applyNameTranslator: classifier does not get translated.");
  //classifier_->applyNameTranslator(id, translator);
  trainer_->applyNameTranslator(id, translator);
  ROS_ASSERT(incoming_annotated_.empty());
  annotated_->applyNameTranslator(id, translator);
  autobg_->applyNameTranslator(id, translator);
  unsupervised_->applyNameTranslator(id, translator);
  validation_->applyNameTranslator(id, translator);
  if(test_)
    test_->applyNameTranslator(id, translator);
  
  if(id == "cmap") {
    stats_.applyNameTranslator(id, translator);
  }

  hand_mutex_.unlock();
}

void OnlineLearner::serialize(std::ostream& out) const
{
  scopeLockRead;
  boost::unique_lock<boost::shared_mutex> ulock(hand_mutex_);

  out << "OnlineLearner v0.2" << endl;
  serializeNameMappings(out);

  // -- Algorithm params
  eigen_extensions::serializeScalar(emax_, out);
  eigen_extensions::serializeScalar(buffer_size_, out);
  eigen_extensions::serializeScalar(max_track_length_, out);

  // -- Algorithm data
  out << *classifier_;  
  out << *trainer_;
  eigen_extensions::serializeScalar(incoming_annotated_.size(), out);
  for(size_t i = 0; i < incoming_annotated_.size(); ++i)
    out << *incoming_annotated_[i];
  out << *annotated_;
  out << *autobg_;
  out << *unsupervised_;
  out << *validation_;
  eigen_extensions::serializeScalar(ann_counter_, out);
  for(size_t i = 0; i < unsupervised_logodds_.size(); ++i)
    out << unsupervised_logodds_[i];

  // -- Bookkeeping params
  eigen_extensions::serializeScalar(max_iters_, out);
  eigen_extensions::serializeScalar(snapshot_every_, out);
  eigen_extensions::serializeScalar(evaluate_every_, out);
  out << output_dir_ << endl;
  out << unlabeled_dir_ << endl;
  out << saved_annotations_dir_ << endl;

  // -- Bookkeeping data
  out << stats_;
  eigen_extensions::serializeScalar(iter_, out);
  out << td_path_ << endl;
}

void OnlineLearner::deserialize(std::istream& in)
{
  scopeLockWrite;
  boost::unique_lock<boost::shared_mutex> ulock(hand_mutex_);

  string buf;
  getline(in, buf);
  ROS_ASSERT(buf == "OnlineLearner v0.2");

  deserializeNameMappings(in);

  // -- Algorithm params
  eigen_extensions::deserializeScalar(in, &emax_);
  eigen_extensions::deserializeScalar(in, &buffer_size_);
  eigen_extensions::deserializeScalar(in, &max_track_length_);

  // -- Algorithm data
  classifier_ = GridClassifier::Ptr(new GridClassifier);
  in >> *classifier_;
  trainer_ = GridClassifier::BoostingTrainer::Ptr(new GridClassifier::BoostingTrainer(classifier_));
  in >> *trainer_;
  size_t num_incoming_annotated;
  eigen_extensions::deserializeScalar(in, &num_incoming_annotated);
  incoming_annotated_.resize(num_incoming_annotated);
  for(size_t i = 0; i < incoming_annotated_.size(); ++i) {
    incoming_annotated_[i] = TrackDataset::Ptr(new TrackDataset);
    in >> *incoming_annotated_[i];
  }
  annotated_ = TrackDataset::Ptr(new TrackDataset);
  in >> *annotated_;
  autobg_ = TrackDataset::Ptr(new TrackDataset);
  in >> *autobg_;
  unsupervised_ = TrackDataset::Ptr(new TrackDataset);
  in >> *unsupervised_;
  validation_ = TrackDataset::Ptr(new TrackDataset);
  in >> *validation_;
  eigen_extensions::deserializeScalar(in, &ann_counter_);

  unsupervised_logodds_.resize(unsupervised_->size());
  for(size_t i = 0; i < unsupervised_logodds_.size(); ++i)
    in >> unsupervised_logodds_[i];

  viewable_unsupervised_ = TrackDataset::Ptr(new TrackDataset);
  
  // -- Bookkeeping params
  eigen_extensions::deserializeScalar(in, &max_iters_);
  eigen_extensions::deserializeScalar(in, &snapshot_every_);
  eigen_extensions::deserializeScalar(in, &evaluate_every_);
  getline(in, output_dir_);
  getline(in, unlabeled_dir_);
  getline(in, saved_annotations_dir_);
  input_auto_annotations_dir_ = output_dir_ + "/input_auto_annotations";
  input_hand_annotations_dir_ = output_dir_ + "/input_hand_annotations";

  // -- Bookkeeping data
  in >> stats_;
  eigen_extensions::deserializeScalar(in, &iter_);
  ++iter_;
  getline(in, td_path_);
  paused_ = false;
}

TrackDataset OnlineLearner::requestInductedSample(const std::string& cname,
                                                  float prediction, size_t num) const
{
  boost::unique_lock<boost::shared_mutex> ulock(viewable_unsupervised_mutex_);

  if(!nameMapping("cmap").hasName(cname)) {
    ROS_WARN_STREAM("OnlineLearner got a request for inducted tracks of class \""
                    << cname << "\", but that name does not exist in the cmap.");
    TrackDataset td;
    return td;
  }
  size_t cidx = nameMapping("cmap").toId(cname);
  
  // -- Sort tracks by how close they are to the prediction we want.
  const TrackDataset& vuns = *viewable_unsupervised_;
  vector< pair<double, size_t> > index;
  index.reserve(vuns.size());
  for(size_t i = 0; i < vuns.size(); ++i) {
    Label pred = vuns.label(i);
    // Tracks with a label of exactly zero are those that were de-inducted by
    // the system for a reason.  They should be ignored.
    if(pred(cidx) == 0)
      continue;
    index.push_back(pair<double, size_t>(fabs(pred(cidx) - prediction), i));
  }
  sort(index.begin(), index.end());  // ascending

  // -- Set up the new TD.
  TrackDataset td;
  td.tracks_.resize(min(index.size(), num));
  for(size_t i = 0; i < td.tracks_.size(); ++i)
    td.tracks_[i] = Dataset::Ptr(new Dataset);
  td.applyNameMappings(vuns);

  // -- Copy over the tracks.
  for(size_t i = 0; i < td.size(); ++i)
    td[i] = vuns[index[i].second];

  // Clear the dmap for the outgoing tracks since they do not include descriptors.
  // TODO: This should happen for vuns, too, ... but we'll be getting rid of that.
  td.applyNameMapping("dmap", NameMapping());

  requestInductedSampleHook(&td, cidx);
  
  return td;
}

void OnlineLearner::entryHook(TrackDataset* td, const std::string& path) const
{
  ROS_ASSERT(nameMappingsAreEqual(*td));
}

TrackDataset::Ptr OnlineLearner::loadTrackDataset(const std::string& path) const
{
  TrackDataset::Ptr td(new TrackDataset);
  td->load(path);
  entryHook(td.get(), path);
  return td;
}

/************************************************************
 * Stats
 ************************************************************/

OnlineLearner::Stats::Stats() :
  elapsed_(0),
  num_unl_frames_seen_(0),
  num_unl_tracks_seen_(0),
  total_frames_annotated_(0),
  total_tracks_annotated_(0)
{
}

void OnlineLearner::Stats::incrementUnlabeled(const TrackDataset& dataset)
{
  num_unl_tracks_seen_ += dataset.size();
  num_unl_frames_seen_ += dataset.totalInstances();
}

void OnlineLearner::Stats::incrementAnnotation(const TrackDataset& dataset)
{
  for(size_t i = 0; i < dataset.size(); ++i) {
    const Dataset& track = dataset[i];
    Label trl = dataset.label(i);
    bool any = false;
    for(int j = 0; j < trl.rows(); ++j) {
      if(trl(j) > 0) {
	any = true;
	num_frames_annotated_pos_(j) += track.size();
	++num_tracks_annotated_pos_(j);
      }
      else if(trl(j) < 0) {
	any = true;
	num_frames_annotated_neg_(j) += track.size();
	++num_tracks_annotated_neg_(j);
      }
    }

    if(any) {
      total_frames_annotated_ += track.size();
      ++total_tracks_annotated_;
    }
  }
}

std::string OnlineLearner::Stats::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "Total elapsed time (seconds): " << elapsed_ << endl;
  oss << prefix << "Total unlabeled frames looked at: " << num_unl_frames_seen_ << endl;
  oss << prefix << "Total unlabeled tracks looked at: " << num_unl_tracks_seen_ << endl;
  oss << prefix << "Total frames annotated: " << total_frames_annotated_ << endl;
  oss << prefix << "Total tracks annotated: " << total_tracks_annotated_ << endl;

  oss << prefix << "Per-class stats: " << endl;
  NameMapping cmap = nameMapping("cmap");
  for(size_t i = 0; i < cmap.size(); ++i) {
    oss << prefix << "  " << cmap.toName(i) << " stats" << endl;
    oss << prefix << "    Num frames annotated as " << cmap.toName(i) << " (pos, neg): " << num_frames_annotated_pos_(i) << " " << num_frames_annotated_neg_(i) << endl;
    oss << prefix << "    Num tracks annotated as " << cmap.toName(i) << " (pos, neg): " << num_tracks_annotated_pos_(i) << " " << num_tracks_annotated_neg_(i) << endl;
  }

  return oss.str();
}

void OnlineLearner::Stats::serialize(std::ostream& out) const
{
  out << "OnlineLearner::Stats v0.2" << endl;
  serializeNameMappings(out);
  eigen_extensions::serializeScalar(elapsed_, out);
  eigen_extensions::serializeScalar(num_unl_frames_seen_, out);
  eigen_extensions::serializeScalar(num_unl_tracks_seen_, out);
  eigen_extensions::serializeScalar(total_frames_annotated_, out);
  eigen_extensions::serializeScalar(total_tracks_annotated_, out);
  eigen_extensions::serialize(num_frames_annotated_pos_, out);
  eigen_extensions::serialize(num_tracks_annotated_pos_, out);
  eigen_extensions::serialize(num_frames_annotated_neg_, out);
  eigen_extensions::serialize(num_tracks_annotated_neg_, out);
}

void OnlineLearner::Stats::deserialize(std::istream& in)
{
  string buf;
  getline(in, buf);
  ROS_ASSERT(buf == "OnlineLearner::Stats v0.2");
  
  deserializeNameMappings(in);
  eigen_extensions::deserializeScalar(in, &elapsed_);
  eigen_extensions::deserializeScalar(in, &num_unl_frames_seen_);
  eigen_extensions::deserializeScalar(in, &num_unl_tracks_seen_);
  eigen_extensions::deserializeScalar(in, &total_frames_annotated_);
  eigen_extensions::deserializeScalar(in, &total_tracks_annotated_);
  eigen_extensions::deserialize(in, &num_frames_annotated_pos_);
  eigen_extensions::deserialize(in, &num_tracks_annotated_pos_);
  eigen_extensions::deserialize(in, &num_frames_annotated_neg_);
  eigen_extensions::deserialize(in, &num_tracks_annotated_neg_);
}

void OnlineLearner::Stats::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  ROS_ASSERT(id == "cmap");
  
  translator.translate(&num_frames_annotated_pos_, 0);
  translator.translate(&num_tracks_annotated_pos_, 0);
  translator.translate(&num_frames_annotated_neg_, 0);
  translator.translate(&num_tracks_annotated_neg_, 0);
}

/************************************************************
 * Helper functions
 ************************************************************/

void getNextPath(std::string dir, std::string ext, std::string* path) 
{
  // -- Get contents of dir in order.
  vector<string> file_paths;
  bfs::recursive_directory_iterator it(dir), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    // Ignore hidden files.  If you are saving new TD files while group induction is running,
    // this will prevent you from loading partially-saved files.
    if(is_regular_file(p) && bfs::extension(p).compare(ext) == 0 && p.leaf().string()[0] != '.')
      file_paths.push_back(p.string());
  }
  sort(file_paths.begin(), file_paths.end());
  ROS_ASSERT(!file_paths.empty());
  
  // -- Get the next path deterministically.
  //    If we can't find the current path, start at the beginning.
  //    If we're at the end, wrap around to the beginning.
  //    Otherwise just go to the next one.
  
  // vector<string>::iterator vit = find(file_paths.begin(), file_paths.end(), *path);
  // if(vit == file_paths.end()) {
  //   vit = file_paths.begin();
  //   ROS_DEBUG_STREAM("Could not find unlabeled td path \"" << *path << "\".  Starting from the beginning.");
  // }
  // else if(vit == file_paths.end() - 1) {
  //   ROS_DEBUG("Wrapping around unlabeled tds.");
  //   vit = file_paths.begin();
  // }
  // else
  //   ++vit;

  // *path = *vit;

  
  // -- Choose a random path.
  *path = file_paths[rand() % file_paths.size()];

  cout << "Next unlabeled dataset: " << *path << endl;
}

ostream& operator<<(ostream& out, const vector<string>& strings)
{
  for(size_t i = 0; i < strings.size(); ++i)
    out << "  " << strings[i] << endl;

  return out;
}

