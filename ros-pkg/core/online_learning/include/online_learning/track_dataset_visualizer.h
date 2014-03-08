#ifndef TRACK_DATASET_VISUALIZER_H
#define TRACK_DATASET_VISUALIZER_H

#include <online_learning/tbssl.h>

#include <ros/assert.h>
#include <ros/console.h>
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

class ClusterViewController;
class TrackDataset;

class TrackView
{
public:
  virtual ~TrackView() {}
  
  virtual void displayInstance(Instance& instance, void* caller) = 0;
  virtual void clearInstance(void* caller) = 0;
  virtual void displayMessage(const std::string& message, void* caller) = 0;
  //! Returns true if there is a new keypress to act on.
  virtual bool keypress(pcl::visualization::KeyboardEvent* event, void* caller) = 0;
};

// Interface for any class that is used for viewing c   lusters of tracks.
class ClusterView
{
public:
  virtual ~ClusterView() {}
  // TD contains shared_ptrs.
  virtual void displayCluster(boost::shared_ptr<TrackDataset> td) = 0;
  virtual void displayMessage(const std::string& message) = 0;
};


class TrackViewControllerBase : public Agent
{
public:
  //! Set this if a user is to be able to use 'S' to save the TrackDataset to disk.
  std::string write_path_;

  //! delay is in ms.
  TrackViewControllerBase(TrackView* view, int delay = 30);
  TrackViewControllerBase(TrackView* view, ClusterView *cview, int delay = 30);
  virtual ~TrackViewControllerBase() {}
  //! TODO: Make this only usable when not running.
  virtual void setTrackDataset(TrackDataset::Ptr td);
    
protected:
  OnlineLearner* learner_;
  TrackView* view_;
  ClusterView *cview_;
  TrackDataset::Ptr td_;
  //! Indexes into index_.
  int tidx_;
  int fidx_;
  //! td_->track_[index_[tidx_]] is the track we are currently looking at.
  //! index_ lets you sort however you want.
  std::vector<int> index_;
  int sort_class_;
  Label to_apply_;
  int delay_;

  virtual bool handleKeypress(const pcl::visualization::KeyboardEvent& event);
  virtual void incrementTrack(int val);
  virtual void incrementFrame(int val);
  virtual void _run();
  virtual std::string message();
  virtual void updateDisplay();
  virtual void handleClassKeypress(int c, bool alt);
  virtual void saveDataset() const;
  virtual void applyLabel();
  
  //! Override this to change how you are sorting.
  //! By default there is no sorting.
  //! You must call this function manually whenever td_ changes.
  virtual void updateIndex();
  void updateIndexDefault();
  //! Sort by label_(c) in descending order.
  void updateIndexByClass(size_t c);
  void rotateSortClass(int val);
  int classKeyToIdx(char key) const;
  char classIdxToKey(int idx) const;
};

class ActiveLearningViewController : public TrackViewControllerBase
{
public:
  ActiveLearningViewController(TrackView* view, ClusterView *cview,
                               OnlineLearner* learner,
                               std::string unlabeled_td_dir);
  virtual ~ActiveLearningViewController() {}

protected:
  OnlineLearner* learner_;
  std::string unlabeled_td_dir_;
  std::string next_path_;
  std::string current_path_;
  
  GridClassifier gc_;
  //! How long it's been since the classifier has been updated.
  HighResTimer hrt_classifier_;
  
  bool handleKeypress(const pcl::visualization::KeyboardEvent& event);
  std::string message();
  void applyLabel();
  void getNextUnlabeledDatasetPath();
  void loadNextUnlabeledDataset();
  void ClusterSimilarTracks(TrackDataset *new_td,
                            const Dataset &ref);
};

class InductionViewController : public TrackViewControllerBase
{
public:
  InductionViewController(OnlineLearner* learner, TrackView* view);
  
protected:
  OnlineLearner* learner_;
  std::vector<double> hashes_;
  //! Keeps track of how long it has been since you updated.
  HighResTimer hrt_;

  std::string message();
  bool handleKeypress(const pcl::visualization::KeyboardEvent& event);
  void applyLabel();
  void updateUnsupervisedDataset();
};

class DGCTrackView : public TrackView, public Agent
{
public:
  DGCTrackView();
  virtual ~DGCTrackView();
  void displayInstance(Instance& instance, void* caller);
  void clearInstance(void* caller);
  void displayMessage(const std::string& message, void* caller);
  bool keypress(pcl::visualization::KeyboardEvent* event, void* caller);
  
  void _run();
  
protected:
  typedef pcl::visualization::PCLVisualizer PCLVisualizer;
  PCLVisualizer* visualizer_;
  bool needs_update_;
  Cloud::Ptr vis_;
  std::string message_;
  std::vector<pcl::visualization::KeyboardEvent> events_;

  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
};

class VCMultiplexor : public TrackView, public SharedLockable
{
public:
  VCMultiplexor(TrackView* view);
  virtual ~VCMultiplexor() {}
  void addVC(void* address);
  virtual void displayInstance(Instance& instance, void* caller);
  virtual void clearInstance(void* caller);
  virtual void displayMessage(const std::string& message, void* caller);
  virtual bool keypress(pcl::visualization::KeyboardEvent* event, void* caller);
  
protected:
  TrackView* view_;
  std::vector<void*> vcs_;
  size_t active_;
};

#endif // TRACK_DATASET_VISUALIZER_H
