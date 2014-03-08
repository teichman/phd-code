#include <online_learning/track_dataset_visualizer.h>
#include <online_learning/clusterer.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

/************************************************************
 * TrackViewControllerBase
 ************************************************************/

TrackViewControllerBase::TrackViewControllerBase(TrackView* view, int delay) :
  learner_(NULL),
  view_(view),
  cview_(NULL),
  td_(new TrackDataset),
  tidx_(0),
  fidx_(0),
  sort_class_(0),
  delay_(delay)
{
  ROS_ASSERT(view_);
}

TrackViewControllerBase::TrackViewControllerBase(TrackView* view,
                                                 ClusterView *cview, int delay) :
  learner_(NULL),
  view_(view),
  cview_(cview),
  td_(new TrackDataset),
  tidx_(0),
  fidx_(0),
  sort_class_(0),
  delay_(delay)
{
  ROS_ASSERT(view_);
}

void TrackViewControllerBase::setTrackDataset(TrackDataset::Ptr td)
{
  td_ = td;
  updateIndex();
//  if (cview_ != NULL) {
//    cview_->displayCluster(td_);
//  }
}


void TrackViewControllerBase::handleClassKeypress(int c, bool alt)
{
  if(!td_->hasNameMapping("cmap"))
    return;

  if((size_t)to_apply_.rows() != td_->nameMapping("cmap").size())
    to_apply_ = VectorXf::Zero(td_->nameMapping("cmap").size());

  to_apply_(c) += 1;
  if(to_apply_(c) > 1 + 1e-6)
    to_apply_(c) = -1;

  if(alt) {
    to_apply_.setConstant(-1);
    to_apply_(c) = 1;
  }
  
  // if(alt)
  //   to_apply_(c) = -1;
  // else
  //   to_apply_(c) = 1;
}

bool TrackViewControllerBase::handleKeypress(const pcl::visualization::KeyboardEvent& event)
{
  // Shift-return to send label.
  if(event.isShiftPressed() && event.getKeyCode() == 13) {
    applyLabel();
    return true;
  }
  
  // -- If it's a class keypress, handle that specially.
  char key = event.getKeyCode();
  int ckey = classKeyToIdx(key);
  if(td_->hasNameMapping("cmap") && ckey >= 0 && ckey < (int)td_->nameMapping("cmap").size()) {
    handleClassKeypress(ckey, event.isAltPressed());
    return true;
  }

  // -- Otherwise check for standard options.
  bool used = true;
  switch(key) {
  case ',':
    incrementTrack(-1);
    break;
  case '.':
    incrementTrack(1);
    break;
  case '<':
    incrementTrack(-100);
    break;
  case '>':
    incrementTrack(100);
    break;
  case '{':
    rotateSortClass(-1);
    break;
  case '}':
    rotateSortClass(1);
    break;
  case 'S':
    saveDataset();
    break;
  default:
    used = false;
    break;
  }

  return used;
}

void TrackViewControllerBase::saveDataset() const
{
  if(write_path_ == "") {
    cout << "[TrackViewControllerBase] Can only saveDataset when write_path_ is set." << endl;
    return;
  }

  cout << "[TrackViewControllerBase] Writing TrackDataset to " << write_path_ << endl;
  td_->save(write_path_);
  cout << "[TrackViewControllerBase] Done." << endl;
}

void TrackViewControllerBase::applyLabel()
{
  if(!td_->hasNameMapping("cmap") || to_apply_.rows() == 0) {
    ROS_WARN("In TrackViewControllerBase::applyLabel, but cmap doesn't exist or to_apply_ is empty.");
    return;
  }

  (*td_)[index_[tidx_]].setLabel(to_apply_);
}

void TrackViewControllerBase::rotateSortClass(int val)
{
  if(!td_->hasNameMapping("cmap"))
    return;
  
  sort_class_ += val;
  if(sort_class_ < 0)
    sort_class_ = td_->nameMapping("cmap").size() - 1;
  
  sort_class_ = sort_class_ % td_->nameMapping("cmap").size();
  updateIndex();
  tidx_ = 0;
  fidx_ = (*td_)[index_[tidx_]].size() / 2;
}

void TrackViewControllerBase::incrementTrack(int val)
{
  if(td_->empty())
    return;
  
  tidx_ += val;

  if(tidx_ < 0)
    tidx_ = 0;
  if(tidx_ >= (int)index_.size())
    tidx_ = index_.size() - 1;

  fidx_ = (*td_)[index_[tidx_]].size() / 2;
}

void TrackViewControllerBase::incrementFrame(int val)
{
  if(td_->empty())
    return;

  int num_frames = (*td_)[index_[tidx_]].size();
  
  fidx_ += val;
  if(fidx_ < 0)
    fidx_ = num_frames - 1;
  if(fidx_ >= num_frames)
    fidx_ = 0;
}

void TrackViewControllerBase::_run()
{
  while(true) {
    incrementFrame(1);
    
    pcl::visualization::KeyboardEvent event(true, " ", ' ', false, false, false);  // Throwaway key.
    if(view_->keypress(&event, this))
      handleKeypress(event);
    
    updateDisplay();
    usleep(delay_ * 1e3);
  }
}

char TrackViewControllerBase::classIdxToKey(int idx) const
{
  return idx + 'a';
}

int TrackViewControllerBase::classKeyToIdx(char key) const
{
  return key - 'a';
}

std::string TrackViewControllerBase::message()
{
  ostringstream oss;
  oss << "TrackViewControllerBase" << endl;

  if(td_->hasNameMapping("cmap")) {
    oss << "Annotation (shift-return to apply): " << endl;
    if(to_apply_.rows() == 0)
      oss << "  None." << endl;
    else
      for(int c = 0; c < to_apply_.rows(); ++c)
        oss << "  (" << classIdxToKey(c) << ") "
            << td_->nameMapping("cmap").toName(c)
            << ": " << to_apply_(c) << endl;
  }
  
  if(td_->empty()) {
    oss << "No tracks yet." << endl;
    return oss.str();
  }

  ROS_ASSERT(tidx_ < (int)index_.size());
  ROS_ASSERT(index_[tidx_] < (int)td_->size());
  const Dataset& track = (*td_)[index_[tidx_]];
  oss << "Track: " << tidx_ << " / " << index_.size() << endl;
  
  if(track.empty()) {
    oss << "Empty track." << endl;
    return oss.str();
  }

  ROS_ASSERT(fidx_ >= 0 && fidx_ < (int)track.size());
  const Instance& frame = track[fidx_];
  
  oss << "Frame: " << fidx_ << " / " << track.size() << endl;
  oss << "Label: " << endl;
  for(size_t c = 0; c < td_->nameMapping("cmap").size(); ++c) {
    oss << "  (" << classIdxToKey(c) << ") " << td_->nameMapping("cmap").toName(c) << ": " << frame.label_(c) << endl;
  }

  oss << "Sort class: " << td_->nameMapping("cmap").toName(sort_class_);
  
  return oss.str();
}

void TrackViewControllerBase::updateDisplay()
{
  view_->displayMessage(message(), this);

  // -- Get the next frame, if there is one.
  //    Clear the display if not.
  if(td_->empty()) {
    view_->clearInstance(this);
    return;
  }

  ROS_ASSERT(tidx_ < (int)index_.size());
  ROS_ASSERT(index_[tidx_] < (int)td_->size());
  Dataset& track = (*td_)[index_[tidx_]];
  
  if(track.empty()) {
    view_->clearInstance(this);
    return;
  }

  ROS_ASSERT(fidx_ >= 0 && fidx_ < (int)track.size());
  view_->displayInstance(track[fidx_], this);
}

void TrackViewControllerBase::updateIndex()
{
  if(!td_->hasNameMapping("cmap"))
    return;
  
  //updateIndexDefault();
  ROS_ASSERT(sort_class_ >= 0 && sort_class_ < (int)td_->nameMapping("cmap").size());
  updateIndexByClass(sort_class_);
}

void TrackViewControllerBase::updateIndexDefault()
{
  index_.clear();
  index_.reserve(td_->size());
  for(size_t i = 0; i < td_->size(); ++i) {
    // Don't put empty tracks in the index.
    // This gives us a convenient way to mark tracks as
    // already labeled.
    if(td_->tracks_[i]->empty())
      continue;

    index_.push_back(i);
  }
}

void TrackViewControllerBase::updateIndexByClass(size_t c)
{
  vector< pair<double, size_t> > ind;
  ind.reserve(td_->size());
  for(size_t i = 0; i < td_->size(); ++i) {
    // Don't put empty tracks in the index.
    // This gives us a convenient way to mark tracks as
    // already labeled.
    if(td_->tracks_[i]->empty())
      continue;

    Label label = td_->label(i);
    ROS_ASSERT(label.rows() != 0);
    ROS_ASSERT(c < (size_t)label.rows());
    ind.push_back(pair<double, size_t>(label(c), i));
  }
  sort(ind.begin(), ind.end(), greater< pair<double, size_t> >());  // descending

  index_.clear();
  index_.resize(ind.size());
  for(size_t i = 0; i < ind.size(); ++i)
    index_[i] = ind[i].second;

  incrementTrack(0);
}


/************************************************************
 * InductionViewController
 ************************************************************/

InductionViewController::InductionViewController(OnlineLearner* learner, TrackView* view) :
  TrackViewControllerBase(view),
  learner_(learner)
{
}

std::string InductionViewController::message()
{
  ostringstream oss;

  oss << "InductionViewController" << endl;
  oss << "OnlineLearner is ";
  if(learner_->paused())
    oss << "paused." << endl;
  else
    oss << "running." << endl;
  
  oss << "Last update: " << (int)hrt_.getMinutes() << " minutes ago." << endl;

  oss << endl;
  oss << TrackViewControllerBase::message() << endl;
  
  return oss.str();
}

void InductionViewController::applyLabel()
{
  if(!td_->hasNameMapping("cmap") || to_apply_.rows() == 0) {
    ROS_WARN("In InductionViewController::applyLabel, but cmap doesn't exist or to_apply_ is empty.");
    return;
  }

  TrackDataset::Ptr td(new TrackDataset);
  ROS_ASSERT(learner_->nameMappingsAreEqual(*td_));
  td->applyNameMappings(*td_);
  Dataset::Ptr track = td_->copy(index_[tidx_]);

  // Ensure computation of descriptors by Inductor::entryHook.
  // TODO: td_, which comes from viewableUnsupervised in OnlineLearner,
  //   should probably have an empty dmap.  Is there some reason this
  //   isn't the case?
  td->applyNameMapping("dmap", NameMapping());
  track->applyNameMapping("dmap", NameMapping());

  track->setLabel(to_apply_);
  td->tracks_.push_back(track);
  ROS_ASSERT(td->nameMappingsAreEqual(*td->tracks_[0]));

  //learner_->pushHandLabeledDataset(td);
  // The boost::thread will destruct at the end of this function.
  // This detaches the thread - it continues to run until pushHandLabeledDataset is complete,
  // even though the boost::thread object is gone.
  // http://www.boost.org/doc/libs/1_55_0/doc/html/thread/thread_management.html#thread.thread_management.tutorial.attributes
  boost::thread pusher(&OnlineLearner::pushHandLabeledDataset, learner_, td);
  
  // Don't show this track anymore.
  (*td_)[index_[tidx_]].instances_.clear();
  updateIndex();
  
  cout << "[InductionViewController]  Sent track to learner with annotation " << to_apply_.transpose() << endl;
}

void InductionViewController::updateUnsupervisedDataset()
{
  cout << "[InductionViewController]  Getting new snapshot of the unsupervised dataset..." << endl;
  while(true) { 
    learner_->viewableUnsupervised(td_.get(), &hashes_);
    ROS_ASSERT(td_->size() == hashes_.size());
    if(td_->empty()) {
      cout << "[InductionViewController]  Unsupervised dataset is empty.  Waiting..." << endl;
      usleep(5e6);
    }
    else
      break;
  }
  updateIndex();
  incrementTrack(0);  // Make sure we're not looking past the end.
  to_apply_ = VectorXf::Zero(td_->nameMapping("cmap").size());
  hrt_.reset();
  hrt_.start();

  set<double> hashset;
  for(size_t i = 0; i < hashes_.size(); ++i)
    hashset.insert(hashes_[i]);
  if(hashset.size() < hashes_.size())
    ROS_WARN_STREAM("[InductionViewController]  " << hashes_.size() - hashset.size() << " hash collisions detected." << std::flush);
    
  cout << "[InductionViewController]  Done." << endl;
}

bool InductionViewController::handleKeypress(const pcl::visualization::KeyboardEvent& event)
{
  if(TrackViewControllerBase::handleKeypress(event))
    return true;
  
  bool used = true;
  char key = event.getKeyCode();
  switch(key) {
  case 'I':
    updateUnsupervisedDataset();
    break;
  case ' ':
    learner_->togglePaused();
    break;
  case '!':
    cout << "Requesting snapshot..." << endl;
    learner_->request_snapshot_ = true;
    break;
  default:
    used = false;
    break;
  }

  return used;
}


/************************************************************
 * ActiveLearningViewController
 ************************************************************/

std::string datasetPathToName(std::string path)
{
  bfs::path p(path);
  return p.filename().string();
}

ActiveLearningViewController::ActiveLearningViewController(
    TrackView* view, ClusterView *cview, OnlineLearner* learner,
    std::string unlabeled_td_dir) :
    TrackViewControllerBase(view, cview), learner_(learner), unlabeled_td_dir_(
        unlabeled_td_dir)
{
}

void ActiveLearningViewController::getNextUnlabeledDatasetPath()
{
  // -- Get contents of unlabeled_td_dir in order.
  vector<string> td_paths;
  bfs::recursive_directory_iterator it(unlabeled_td_dir_), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    if(is_regular_file(p) && bfs::extension(p).compare(".td") == 0)
      td_paths.push_back(p.string());
  }
  sort(td_paths.begin(), td_paths.end());
  ROS_ASSERT(!td_paths.empty());
  
  // -- Choose one at random.
  next_path_ = td_paths[rand() % td_paths.size()];
  cout << "[ActiveLearningViewController]  Next dataset name: " << datasetPathToName(next_path_) << endl;
}

void ActiveLearningViewController::loadNextUnlabeledDataset()
{
  if(next_path_ == "") {
    cout << "[ActiveLearningViewController]  You must select a new TD file with 'n' first." << endl;
    return;
  }
  
  // -- Load the next track manager.
  current_path_ = next_path_;

  cout << "[ActiveLearningViewController]  Loading " << datasetPathToName(current_path_) << "." << endl;
  TrackDataset::Ptr td = learner_->loadTrackDataset(current_path_);

  // If this dataset is labeled, throw away the labels.
  for(size_t i = 0; i < td->size(); ++i)
    for(size_t j = 0; j < (*td)[i].size(); ++j)
      (*td)[i][j].label_.setZero();
    
  cout << "[ActiveLearningViewController]  Loaded " << td->size() << " tracks." << endl;
  ROS_ASSERT(!td->tracks_.empty());

  // -- Get the most recent classifier.
  learner_->copyClassifier(&gc_);
  hrt_classifier_.stop();
  hrt_classifier_.reset();
  hrt_classifier_.start();
      
  // -- Classify max 100 frames of everything.
  cout << "[ActiveLearningViewController]  classifying new dataset..." << endl;
  #pragma omp parallel for
  for(size_t i = 0; i < td->size(); ++i) {
    Dataset& track = (*td)[i];
    if(track.empty())
      continue;
    size_t incr = (double)track.size() / 100.;
    incr = max(incr, (size_t)1);
    
    Label track_prediction = VectorXf::Zero(gc_.nameMapping("cmap").size());
    double num = 0;
    for(size_t j = 0; j < track.size(); j += incr) {
      track_prediction += gc_.classify(track[j]);
      ++num;
    }

    track_prediction /= num;
    track.setLabel(track_prediction);
  }
  cout << "[ActiveLearningViewController]  done classifying new dataset." << endl;

  setTrackDataset(td);
  getNextUnlabeledDatasetPath();
}

void ActiveLearningViewController::ClusterSimilarTracks(TrackDataset *new_td,
                                                        const Dataset &ref) {
  TrackDataset::Ptr clustered_(new TrackDataset());
  cout << "classifying " << new_td->tracks_.size() << " tracks..." << endl;

  int num_added = 0;
  for (size_t i = 0; i < new_td->tracks_.size(); i++)
  {
    ROS_ASSERT(new_td->tracks_[i].get() != NULL);
    if (similar(ref, *new_td->tracks_[i].get(), gc_, 0.7, 3))
    {
      cout << "adding track of size " << new_td->tracks_[i]->size() << endl;
      clustered_->tracks_.push_back(new_td->tracks_[i]);
      num_added++;
    }
  }
  cout << "added " << num_added << " tracks";
  cview_->displayCluster(clustered_);
}


bool ActiveLearningViewController::handleKeypress(const pcl::visualization::KeyboardEvent& event)
{
  if(TrackViewControllerBase::handleKeypress(event))
    return true;
    
  bool used = true;
  char key = event.getKeyCode();
  switch(key) {
  case 'n':
    getNextUnlabeledDatasetPath();
    break;
  case 'N':
    loadNextUnlabeledDataset();
    break;
  case 'c':
    ClusterSimilarTracks(td_.get(), *td_->tracks_[index_[tidx_]].get());
    break;
  case ' ':
    learner_->togglePaused();
    break;
  default:
    used = false;
    break;
  }

  return used;
}

std::string ActiveLearningViewController::message()
{
  ostringstream oss;

  oss << "ActiveLearningViewController" << endl;
  oss << "OnlineLearner is ";
  if(learner_->paused())
    oss << "paused." << endl;
  else
    oss << "running." << endl;
  
  oss << "Last update: " << (int)hrt_classifier_.getMinutes() << " minutes ago." << endl;
  oss << "TrackDataset name: " << datasetPathToName(current_path_) << endl;

  oss << endl;
  oss << TrackViewControllerBase::message() << endl;
  
  return oss.str();
}

void ActiveLearningViewController::applyLabel()
{
  if(!td_->hasNameMapping("cmap") || to_apply_.rows() == 0) {
    ROS_WARN("In ActiveLearningViewController::applyLabel, but cmap doesn't exist or to_apply_ is empty.");
    return;
  }

  TrackDataset::Ptr td(new TrackDataset);
  td->applyNameMappings(gc_);
  td->tracks_.push_back(td_->copy(index_[tidx_]));
  td->tracks_[0]->setLabel(to_apply_);
  ROS_ASSERT(td->nameMappingsAreEqual(*td->tracks_[0]));

  //learner_->pushHandLabeledDataset(td);
  // The boost::thread will destruct at the end of this function.
  // This detaches the thread - it continues to run until pushHandLabeledDataset is complete,
  // even though the boost::thread object is gone.
  // http://www.boost.org/doc/libs/1_55_0/doc/html/thread/thread_management.html#thread.thread_management.tutorial.attributes
  boost::thread pusher(&OnlineLearner::pushHandLabeledDataset, learner_, td);
  
  // Don't show this track anymore.
  (*td_)[index_[tidx_]].instances_.clear();
  updateIndex();
  incrementTrack(-1);
  
  cout << "[ActiveLearningViewController]  Sent track to learner with annotation " << to_apply_.transpose() << endl;
}


/************************************************************
 * DGCTrackView
 ************************************************************/

DGCTrackView::DGCTrackView() :
  visualizer_(NULL),
  needs_update_(false),
  vis_(new Cloud)
{
}

DGCTrackView::~DGCTrackView()
{
  if(visualizer_)
    delete visualizer_;
}

void DGCTrackView::_run()
{
  visualizer_ = new PCLVisualizer("Track View");
  
  visualizer_->registerKeyboardCallback(&DGCTrackView::keyboardCallback, *this);
  visualizer_->addCoordinateSystem(1.0);
  visualizer_->setBackgroundColor(0, 0, 0);
  visualizer_->addPointCloud(Cloud::Ptr(new Cloud), "default");
  visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "default");

  while(true) {

    lockWrite();
    if(needs_update_) {
      visualizer_->updatePointCloud(vis_, "default");
      visualizer_->removeShape("text");
      visualizer_->addText(message_, 10, 50, 16, 0.9, 0.9, 0.9, "text");
      needs_update_ = false;
    }
    unlockWrite();
    
    visualizer_->spinOnce(20);
  }

  delete visualizer_;
}

bool DGCTrackView::keypress(pcl::visualization::KeyboardEvent* event, __attribute__((unused)) void* caller)
{
  scopeLockWrite;

  if(events_.empty())
    return false;

  *event = events_.back();
  events_.pop_back();
  return true;
}

void DGCTrackView::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(!event.keyDown())
    return;

  scopeLockWrite;
  events_.clear();  // Turns out having a buffer is really annoying.
  events_.push_back(event);
}

void colorize(int val, Point* pt)
{
  double u = (double)val / 255.;
  pt->g = (0.5 + 0.5 * u) * 255;
  pt->r = val;
  pt->b = val;
}

void DGCTrackView::displayInstance(Instance& instance, __attribute__((unused)) void* caller)
{
  Cloud::Ptr pcd = boost::any_cast<Cloud::Ptr>(instance.raw());
  Cloud::Ptr vis(new Cloud);
  Vector4f centroid;
  pcl::compute3DCentroid(*pcd, centroid);
  pcl::demeanPointCloud(*pcd, centroid, *vis);

  float minval = 100;
  for(size_t i = 0; i < vis->size(); ++i) {
    Point& pt = vis->at(i);
    pt.r = minval + (255.0 - minval) * pt.r / 255.0;
  }

  scopeLockWrite;
  vis_ = vis;
  needs_update_ = true;
}

void DGCTrackView::clearInstance(__attribute__((unused)) void* caller)
{
  scopeLockWrite;
  vis_ = Cloud::Ptr(new Cloud);
  needs_update_ = true;
}

void DGCTrackView::displayMessage(const std::string& message, __attribute__((unused)) void* caller)
{
  scopeLockWrite;
  message_ = message;
  needs_update_ = true;
}

VCMultiplexor::VCMultiplexor(TrackView* view) :
  TrackView(),
  view_(view),
  active_(0)
{
}

void VCMultiplexor::addVC(void* address)
{
  scopeLockWrite;
  vcs_.push_back(address);
}

void VCMultiplexor::displayInstance(Instance& instance, void* caller)
{
  scopeLockWrite;
  
  ROS_ASSERT(active_ < vcs_.size());
  if(caller == vcs_[active_])
    view_->displayInstance(instance, caller);
}

void VCMultiplexor::clearInstance(void* caller)
{
  scopeLockWrite;
  
  ROS_ASSERT(active_ < vcs_.size());
  if(caller == vcs_[active_])
    view_->clearInstance(caller);
}

void VCMultiplexor::displayMessage(const std::string& message, void* caller)
{
  scopeLockWrite;
  
  ROS_ASSERT(active_ < vcs_.size());
  if(caller == vcs_[active_])
    view_->displayMessage(message, caller);
}

bool VCMultiplexor::keypress(pcl::visualization::KeyboardEvent* event, void* caller)
{
  scopeLockWrite;
  
  ROS_ASSERT(active_ < vcs_.size());
  if(caller == vcs_[active_] && view_->keypress(event, caller)) {
    // Intercept Alt-? and don't pass it on to the VC.
    if(event->getKeyCode() == '?' && event->isAltPressed()) {
      ++active_;
      if(active_ == vcs_.size()) 
        active_ = 0;
      return false;  
    }
    // If it's anything else, just pass it on.
    else
      return true;
  }
  else
    return false;
}

