#ifndef TBSSL_H
#define TBSSL_H

// must come first because of python.
#include <online_learning/evaluator.h>  

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <agent/agent.h>
#include <online_learning/grid_classifier.h>
#include <online_learning/training_buffer.h>
#include <online_learning/common.h>

//! ====
//! == Locking strategy:
//! ====
//! The only things that can be accessed and changed by other threads are
//!  * test_                         -- this should only be settable before running.
//!  * max_iters_                    -- this should only be settable before running.
//!  * unlabeled_dir                 -- this should only be settable before running.
//!  * paused_                       -- protected by hand_mutex_.
//!  * incoming_annotated_           -- protected by hand_mutex_.
//!
//! The only things that can be accessed but not changed by other threads
//!  * viewable_unsupervised_        -- protected by viewable_unsupervised_mutex_.
//!  * viewable_unsupervised_hashes_ -- protected by viewable_unsupervised_mutex_.
//!  * vu_incoming_labels_           -- protected by hand_mutex_.
//!  * vu_incoming_hashes_           -- protected by hand_mutex_.
//!  * classifier_                   -- this should not be accessible ever.  Right now ALI is using it, but that's going to change.
class OnlineLearner : public NameMappable, public Serializable, public Agent
{
public:
  class Stats : public Serializable, public NameMappable
  {
  public:
    double elapsed_;
    int num_unl_frames_seen_;
    int num_unl_tracks_seen_;
    int total_frames_annotated_;
    int total_tracks_annotated_;
    Eigen::VectorXi num_frames_annotated_pos_;
    Eigen::VectorXi num_tracks_annotated_pos_;
    Eigen::VectorXi num_frames_annotated_neg_;
    Eigen::VectorXi num_tracks_annotated_neg_;

    Stats();
    void incrementAnnotation(const TrackDataset& dataset);
    void incrementUnlabeled(const TrackDataset& dataset);
    std::string status(const std::string& prefix = "") const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  
  protected:
    void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
  };

  virtual ~OnlineLearner() {}
  //! This is problematic.  If I make serialize and deserialize virtual functions,
  //! then this constructor is totally broken.  Yet I need a way to construct an OL
  //! object from a stream.  Hm.
  //! Leaving serialize and deserialize as non-virtual functions is bad because if you
  //! don't provide them yourself in your subclasses, you'll get the base class one
  //! automatically and this is not necessarily what you want.  This is the least bad
  //! option I can think of at the moment, though.
  OnlineLearner(std::istream& in) { deserialize(in); }
  //! Classifier must be pre-initialized.
  OnlineLearner(double emax,
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


  //! TODO: Make this only callable if OL has not started running.
  void setUnlabeledDir(std::string dir) { unlabeled_dir_ = dir; }
  void setMaxIters(int max_iters) { scopeLockWrite; max_iters_ = max_iters; }
  void setTestData(TrackDataset::Ptr test) { scopeLockWrite; entryHook(test.get()); test_ = test; test_->applyNameMappings(*this); }
  void setPaused(bool val) { boost::unique_lock<boost::shared_mutex> ul(hand_mutex_); paused_ = val; }
  void togglePaused() { boost::unique_lock<boost::shared_mutex> ul(hand_mutex_); paused_ = !paused_; }
  bool paused() const { boost::unique_lock<boost::shared_mutex> ul(hand_mutex_); return paused_; }
  void pushHandLabeledDataset(TrackDataset::Ptr dataset) { entryHook(dataset.get()); boost::unique_lock<boost::shared_mutex> ul(hand_mutex_); incoming_annotated_.push_back(dataset); }
  //! This is separate from pushHandLabeledDataset just so we know how many hand-labeled tracks have
  //! been provided.  We don't need to run this through the usual steps of retrospection, etc,
  //! so we can just add the data directly to the dataset.
  void pushAutoLabeledDataset(TrackDataset::Ptr dataset) { entryHook(dataset.get()); boost::unique_lock<boost::shared_mutex> ul(hand_mutex_); *autobg_ += *dataset; }
  void copyClassifier(GridClassifier* classifier);
  void _run();
  std::string status(const std::string& prefix = "") const;

  //! Makes a copy of viewable_unsupervised_ and viewable_unsupervised_hashes
  //! for a view controller.
  void viewableUnsupervised(TrackDataset* viewable_unsupervised, std::vector<double>* hashes) const;
  //! Puts hash and label in a place where the OL thread will find them eventually.
  //! Then annotateUnsupervised will be called.
  void pushAnnotationForInducted(double hash, const Label& label);

  TrackDataset::ConstPtr autobg() const { return autobg_; }
  TrackDataset::ConstPtr unsupervised() const { return unsupervised_; }
  TrackDataset::ConstPtr annotated() const { return annotated_; }

  //! This function calls entryHook and is used everywhere a new TD is loaded from disk.
  //! Public so that classes like the ActiveLearningViewController can make use of the
  //! same entryHook that OnlineLearner uses.
  TrackDataset::Ptr loadTrackDataset(const std::string& path) const;
  
protected:
  typedef std::vector< std::pair<double, size_t> > ObjectiveIndex;
  
  /************************************************************
   * Algorithm parameters
   ************************************************************/
  //! Worst-case classifier error.  \epsilon_{max} in the math.
  double emax_;
  //! Maximum number of tracks to keep in the unsupervised dataset.
  size_t buffer_size_;
  //! Cut unlabeled tracks such that they are no longer than this.
  size_t max_track_length_;

  /************************************************************
   * Algorithm data
   ************************************************************/
  GridClassifier::Ptr classifier_;
  GridClassifier::BoostingTrainer::Ptr trainer_;
  std::vector<TrackDataset::Ptr> incoming_annotated_;
  TrackDataset::Ptr annotated_;  // The first part of Da
  TrackDataset::Ptr autobg_;  // The second part of Da
  TrackDataset::Ptr unsupervised_;  // Du
  //! If set, will be evaluated on.
  TrackDataset::Ptr test_;
  //! The unsupervised_ dataset contains labels in {-1, 0, +1}.
  //! Here, we store the actual log odds.
  std::vector<Label> unsupervised_logodds_;

  //! This is a copy of unsupervised_ kept specifically for user interaction.
  //! It contains no descriptors so that copying is nearly free.
  //! The labels are set to the log odds rather than the induction label.
  TrackDataset::Ptr viewable_unsupervised_;
  //! Track hashes aligned with viewable_unsupervised_.
  //! Really we should be using unique ids for each unlabeled track,
  //! but I don't have that infrastructure right now.
  std::vector<double> viewable_unsupervised_hashes_;
  //! Protects both viewable_unsupervised_ and viewable_unsupervised_hashes_.
  mutable boost::shared_mutex viewable_unsupervised_mutex_;

  //! Protected by hand_mutex_.
  std::vector<double> vu_incoming_hashes_;
  //! Protected by hand_mutex_.
  std::vector<Label> vu_incoming_labels_;

  
  /************************************************************
   * Bookkeeping params
   ************************************************************/
  int max_iters_;
  int snapshot_every_;
  int evaluate_every_;
  std::string output_dir_;
  std::string unlabeled_dir_;
  //! Looks in saved_annotations_dir_/iterXXXXX for .td files.
  //! If any exist, it will load them as annotated data at the
  //! appropriate iteration.  This is good for setting up test
  //! runs.
  std::string saved_annotations_dir_;
  //! A user can manually add new .td files in this directory.
  //! They will automatically be added to the autobg_ set on
  //! the next iteration.
  std::string input_auto_annotations_dir_;
  //! Like input_auto_annotations_dir_, but for hand annotations.
  std::string input_hand_annotations_dir_;

  /************************************************************
   * Bookkeeping data
   ************************************************************/
  Stats stats_;
  int iter_;
  std::string iter_dir_;  
  mutable boost::shared_mutex hand_mutex_;
  std::string td_path_;
  //! Don't start training until this is false.
  bool paused_;

  /************************************************************
   * Member functions
   ************************************************************/
  void snapshot();
  void evaluate();
  void saveInductionAccuracy(const std::string& basename) const;
  void saveInductionExamples() const;
  void loadSavedAnnotations();
  //! Checks input_hand_annotations_dir_ and input_auto_annotations_dir_ for new .td files.
  void loadInputTDFiles();
  //! dataset will have its track labels set to the post-induction status.
  //! logodds will be filled with track predictions for use later.
  void inductDataset(const Eigen::VectorXf& emin, const Eigen::VectorXf& emax,
                     TrackDataset* dataset, ObjectiveIndex* aggregate_index,
                     std::vector<Label>* logodds,
                     std::vector< std::vector<Label> >* frame_logodds) const;
  void handleAnnotatedData();
  void removePerfectAndNonInducted(TrackDataset* unlabeled_chunk) const;
                                   
  void removeDuplicates(TrackDataset* td) const;
  void updateViewableUnsupervised();
  void inductionStep(TrackDataset* unlabeled_chunk);
  //! De-induct tracks in unsupervised_ so that the ratio of pos : neg is about the same 
  //! as that in annotated_.
  //! Experimental.  Probably shouldn't be used.
  void balance(const ObjectiveIndex& index);
  //! Keep only the most useful tracks in unsupervised_.
  //! This function assumes dual induction and not mutual induction.
  void prune(const ObjectiveIndex& index);
  TrackDataset::Ptr getNextUnlabeledChunk();
  //! Searches the inducted set for any tracks that match hash.
  //! If any exist, they are removed from the inducted set, labeled with label,
  //! and added to incoming_annotated.
  //! Assumes that hand_mutex_ is locked when you call this.
  void annotateUnsupervised(double hash, const Label& label);  
  
  void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);

  //! This function is called any time a new TD enters the OnlineLearner from any source.
  //! This includes the disk and things like pushHandLabeledDataset and pushAutoLabeledDataset.
  //! Subclasses can use this to do things like update descriptors whenever a new TD arrives.
  //! Path is "" if this TD did not come from disk.
  virtual void entryHook(TrackDataset* td, const std::string& path = "") const;
  //! This function is called on the unlabeled chunk just before induction occurs.
  //! You can use it to, for example, remove tracks that don't include enough motion
  //! to be valuable for group induction.
  virtual void chunkHook(TrackDataset* td) const {}
  //! De-induct tracks that the new annotations indicate should not have been inducted.
  //! i.e. "In retrospect, that was wrong..."
  //! By default this uses the worst-case classifier noise method.
  //! To override, de-induct as appropriate in unsupervised_.
  virtual void retrospection(const TrackDataset& new_annotations, const std::vector<Label>& predictions);

private:
  OnlineLearner(const OnlineLearner& other);
  OnlineLearner& operator=(const OnlineLearner& other);
};

void getNextPath(std::string dir, std::string ext, std::string* path);
std::ostream& operator<<(std::ostream& out, const std::vector<std::string>& strings);

#endif // TBSSL_H
