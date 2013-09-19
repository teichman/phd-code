#include <online_learning/active_learning_interface.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;
using namespace pcl::visualization;
using namespace track_manager;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

// ThreadPtr ActiveLearningInterface::create(OnlineLearner* learner, std::string unlabeled_td_dir)
// {
//   ThreadPtr(new boost::thread(boost::bind(&Agent::run, this))); }
//   ActiveLearningInterface ali(learner, unlabeled_td_dir);
// }

float ActiveLearningInterface::confidence(const Label& pred)
{
  return boostingMinConfidence(pred);
}

float ActiveLearningInterface::boostingMinConfidence(const Label& pred)
{
  if(sorting_class_ == -2)
    return pred.array().abs().minCoeff();
  else
    //return fabs(pred(sorting_class_));
    return pred(sorting_class_);
}


ActiveLearningInterface::ActiveLearningInterface(OnlineLearner* learner,
				 std::string unlabeled_td_dir) :
  learner_(learner),
  unlabeled_td_dir_(unlabeled_td_dir),
  track_id_(-1),
  frame_id_(0),
  skip_log_(false),
  sorting_class_(-2),
  paused_(false)
{
}

ActiveLearningInterface::~ActiveLearningInterface()
{
}

void ActiveLearningInterface::_run()
{
  //boost::thread thread_poll(boost::bind(&ActiveLearningInterface::pollingThreadFunction, this));
  //boost::thread thread_cl(boost::bind(&ActiveLearningInterface::classificationThreadFunction, this));

  // -- Apparently PCLVisualizer needs to run in the main thread.
  getNextTrackDatasetPath(&next_td_path_);
  loadNextDataset();
  visualizationThreadFunction();
  // thread_cl.join();
  // thread_poll.join();
}

// void ActiveLearningInterface::pollingThreadFunction()
// {
//   while(!quitting_) {
//     usleep(30e6);

//     GridClassifier* classifier = new GridClassifier;
//     learner_->copyClassifier(classifier);

//     lockRead();
//     bool flag = (!classifier_ || (*classifier != *classifier_));
//     unlockRead();
    
//     if(flag) {
//       ROS_DEBUG("Updating classifier.");
//       lockWrite();
//       if(classifier_) delete classifier_;
//       classifier_ = classifier;
//       clearTracks();
//       unlockWrite();
//     }
//   }
// }

// void ActiveLearningInterface::clearTracks()
// {
//   confidences_.clear();
//   track_id_ = -1;
//   frame_id_ = 0;
// }

void ActiveLearningInterface::reSort()
{
  confidences_.clear();
  confidences_.reserve(td_.size());
  for(size_t i = 0; i < td_.size(); ++i) {
    if(!td_[i].empty()) {
      double conf = confidence(td_.label(i));
      confidences_.push_back(pair<double, size_t>(conf, i));
    }
  }
  sort(confidences_.begin(), confidences_.end());

  incrementTrack(0);
}

void ActiveLearningInterface::loadNextDataset()
{
  // -- Load the next track manager.
  td_path_ = next_td_path_;
  getNextTrackDatasetPath(&next_td_path_);

  cout << "[ActiveLearningInterface]  Loading " << td_path_ << "." << endl;
  td_.load(td_path_);
  cout << "[ActiveLearningInterface]  Next td path is " << next_td_path_ << "." << endl;

  // If this dataset is labeled, throw away the labels.
  for(size_t i = 0; i < td_.size(); ++i)
    for(size_t j = 0; j < td_[i].size(); ++j)
      td_[i][j].label_.setZero();
    
  cout << "[ActiveLearningInterface]  Loaded " << td_.size() << " tracks." << endl;
  ROS_ASSERT(!td_.tracks_.empty());

  // -- Get the most recent classifier.
  learner_->copyClassifier(&gc_);
  classifier_hrt_.stop();
  classifier_hrt_.reset();
  classifier_hrt_.start();
  
  
  // Sometimes the tracks are sorted from longest to shortest.
  // This kind of sucks for active learning, just because the rate
  // at which new tracks are presented is very slow in the beginning
  // and very fast at the end.  So, randomize them.
  //
  // Unfortunately these aren't shared_ptrs anymore.
  // Shuffling the tracks is probably more expensive than we're willing to pay.
  // With the new architecture this really doesn't matter anyway.
  //random_shuffle(td_.tracks_.begin(), td_.tracks_.end());  
  
  // -- Classify max 100 frames of everything.
  cout << "[ActiveLearningInterface]  classifying new dataset..." << endl;
  #pragma omp parallel for
  for(size_t i = 0; i < td_.size(); ++i) {
    Dataset& track = td_[i];
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
  cout << "[ActiveLearningInterface]  done classifying new dataset." << endl;

  reSort();
}

// void ActiveLearningInterface::classificationThreadFunction()
// {
//   lockWrite();
//   getNextTrackManagerPath(&next_td_path_);
//   unlockWrite();
  
//   while(!quitting_) {
//     // -- Load the next track manager.
//     lockWrite();
//     skip_log_ = false;
//     td_path_ = next_td_path_;
//     getNextTrackDatasetPath(&next_td_path_);
//     cout << "Next td path is: " << next_td_path_ << endl;
//     unlockWrite();

//     cout << "Loading " << td_path_ << endl;
//     td_.load(td_path_);
//     // If this dataset is labeled, throw away the labels.
//     for(size_t i = 0; i < td_.size(); ++i)
//       for(size_t j = 0; j < td_[i].size(); ++j)
//         td_[i][j].label_.setZero();
    
//     cout << "Loaded " << td_.size() << " tracks." << endl;
//     ROS_ASSERT(!td_.tracks_.empty());

//     // Sometimes the tracks are sorted from longest to shortest.
//     // This kind of sucks for active learning, just because the rate
//     // at which new tracks are presented is very slow in the beginning
//     // and very fast at the end.  So, randomize them.
//     //
//     // Unfortunately these aren't shared_ptrs anymore.
//     // Shuffling is probably more expensive than we're willing to pay.
//     //random_shuffle(td_.tracks_.begin(), td_.tracks_.end());  

//     // -- Classify max 100 frames of everything.
//     for(size_t i = 0; i < td_.size(); ++i) {
//       lockRead();
//       bool skip_log = skip_log_;
//       unlockRead();
//       if(skip_log)
// 	break;
      
//       Dataset& track = td_[i];
//       if(track.empty())
// 	continue;
//       size_t incr = (double)track.size() / 100.;
//       incr = max(incr, (size_t)1);

//       lockRead();
//       Label track_prediction = VectorXf::Zero(classifier_->nameMapping("cmap").size());
//       for(size_t j = 0; j < track.size(); j += incr)
//         track_prediction += classifier_->classify(track[j]);
//       unlockRead();
//       double conf = confidence(track_prediction);
//       track.setLabel(track_prediction);


      
//       // -- Insert based on confidence.
//       lockWrite();
//       ROS_ASSERT(confidences_.size() == datasets_.size());
//       ROS_ASSERT(confidences_.size() == tracks_.size());
//       size_t idx;
//       for(idx = 0; idx < confidences_.size(); ++idx)
// 	if(conf < confidences_[idx])
//       	  break;
//       ROS_ASSERT(idx <= confidences_.size());
//       confidences_.insert(confidences_.begin() + idx, conf);
//       datasets_.insert(datasets_.begin() + idx, dataset);
//       tracks_.insert(tracks_.begin() + idx, tm.tracks_[i]);
//       ROS_ASSERT(confidences_.size() == datasets_.size());
//       ROS_ASSERT(confidences_.size() == tracks_.size());
      
//       if(track_id_ == -1) {
// 	ROS_ASSERT(tracks_.size() == 1);
// 	track_id_ = 0;
//       }
//       else if(track_id_ >= (int)idx)
// 	++track_id_;
      
//       unlockWrite();
//     }
//   }
// }

void ActiveLearningInterface::visualizationThreadFunction()
{
  // Gross PCLVisualizer camera initialization.
  // Seriously, wtf.
  char* argv[] = {"aoeu", "-cam", "25.7223,47.2289/0,0,0/24.0342,-14.7529,22.264/-0.586992,0.219668,0.779222/0.523599/1600,1280/67,53"};
  int argc = 3;
  PCLVisualizer* viewer = new PCLVisualizer(argc, argv, "Online learning interface");
  viewer->updateCamera();
  viewer->registerKeyboardCallback(&ActiveLearningInterface::keyboardCallback, *this);
  viewer->addCoordinateSystem(1.0);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud(Cloud::Ptr(new Cloud), "default");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 4, "default");

  while(true) {
    { scopeLockRead; if(viewer->wasStopped() || quitting_) break; }
    
    // -- Update track info.
    lockRead();
    ostringstream oss;
    
    if(paused_)
      oss << "OnlineLearner is paused." << endl;
    else
      oss << "OnlineLearner is running." << endl;

    oss << "Current annotation (shift-enter to send): " << endl;
    for(int c = 0; c < current_annotation_.rows(); ++c)
      oss << "  (" << classIdxToKey(c) << ") " << gc_.nameMapping("cmap").toName(c) << ": " << current_annotation_(c) << endl;
    
    if(track_id_ == -1) {
      oss << "No tracks yet." << endl;
    }
    else {
      size_t tidx = confidences_[track_id_].second;
      const Dataset& track = td_[tidx];
      ROS_ASSERT(!track.empty());
      oss << "Track Classification" << endl;
      Label pred = td_.label(tidx);
      for(int c = 0; c < pred.rows(); ++c)
        oss << "  (" << classIdxToKey(c) << ") " << gc_.nameMapping("cmap").toName(c) << ": " << pred(c) << endl;
      
      oss << "Confidence: " << confidences_[track_id_].first << endl;
      oss << "Minutes since last classifier update: " << (int)classifier_hrt_.getMinutes() << endl;
      oss << endl;
      
      oss << "Track " << setw(4) << setfill('0') << track_id_ << " / " << confidences_.size() << endl;
      oss << "Frame " << setw(4) << setfill('0') << frame_id_ << " / " << track.size() << endl;
      oss << endl;
      oss << td_path_ << endl;
    }
    viewer->removeShape("text");
    viewer->addText(oss.str(), 10, 50, 16, 0.9, 0.9, 0.9, "text");
            
    // -- Update visualization cloud.
    Cloud::Ptr vis_pcd(new Cloud);
    if(track_id_ != -1 && frame_id_ != -1) {
      ROS_ASSERT(track_id_ >= 0 && track_id_ < (int)confidences_.size());
      size_t tidx = confidences_[track_id_].second;
      const Dataset& track = td_[tidx];
      //boost::shared_ptr<Frame> frame = boost::any_cast< boost::shared_ptr<Frame> >(track[frame_id_].raw_);
      //frameToCloud(*frame, vis_pcd);
      ROS_FATAL("Update usage of custom data.");
      abort();
    }
    unlockRead();

    // -- Spend some time drawing.
    viewer->updatePointCloud(vis_pcd, "default");
    viewer->spinOnce(20);

    incrementFrame(1);
  }
  

  // -- If we were stopped by 'q', then let other threads know.
  lockWrite();
  quitting_ = true;
  unlockWrite();

  delete viewer;
}

void ActiveLearningInterface::getNextTrackDatasetPath(std::string* path) const
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
  *path = td_paths[rand() % td_paths.size()];
}

bool isShiftedDigit(char key)
{
  if(key == ')' || key == '!' || key == '@')
    return true;
  else
    return false;
}

int atoiShifted(char key)
{
  switch(key) {
  case ')':
    return 0;
    break;
  case '!':
    return 1;
    break;
  case '@':
    return 2;
    break;
  }

  return -1;
}

void ActiveLearningInterface::keyboardCallback(const pcl::visualization::KeyboardEvent& event,
				       void* cookie)
{
  if(!event.keyDown())
    return;
  
  char key = event.getKeyCode();
  string keystr;
  keystr = key;

  // -- Class keypresses.
  int idx = classKeyToIdx(key);
  if(idx >= 0 && idx < (int)gc_.nameMapping("cmap").size()) {
    // -- Negative labeling.
    if(event.isAltPressed()) {
      scopeLockWrite;
      current_annotation_ = VectorXf::Zero(gc_.nameMapping("cmap").size());
      current_annotation_(idx) = -1;
    }
    // -- Positive labeling.
    else {
      scopeLockWrite;
      current_annotation_ = VectorXf::Zero(gc_.nameMapping("cmap").size());
      current_annotation_(idx) = 1;
    }
  }
    
  // -- Other things.
  else {
    switch(key) {
    case 13:
      if(event.isShiftPressed()) {
        if(current_annotation_.rows() != (int)gc_.nameMapping("cmap").size())
          cout << "[ActiveLearningInterface]  Must set current_annotation_ first." << endl;
        else
          labelAs(current_annotation_);
      }
      break;
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
    case '}':
      rotateSortClass(1);
      break;
    case '{':
      rotateSortClass(-1);
      break;
    case 'S':
      loadNextDataset();
      break;
    case 'n':
      lockWrite();
      getNextTrackDatasetPath(&next_td_path_);
      cout << "[ActiveLearningInterface]  Next td path is: " << next_td_path_ << endl;
      unlockWrite();
      break;
    case ' ':
      lockWrite();
      paused_ = !paused_;
      learner_->setPaused(paused_);
      unlockWrite();
      break;
    case 'z':
      lockWrite();
      kickOut();
      unlockWrite();
      break;
    default:
      break;
    }
  }
}

void ActiveLearningInterface::rotateSortClass(int val)
{
  lockWrite();
  sorting_class_ += val;
  sorting_class_ = max(sorting_class_, 0);
  sorting_class_ = min(sorting_class_, (int)gc_.nameMapping("cmap").size() - 1);
  unlockWrite();

  cout << "[ActiveLearningInterface]  Sorting by class " << gc_.nameMapping("cmap").toName(sorting_class_) << "." << endl;
  
  reSort();
}

void ActiveLearningInterface::kickOut()
{
  if(track_id_ == -1)
    return;

  size_t tidx = confidences_[track_id_].second;
  // td_.tracks_.erase(td_.tracks_.begin() + tidx);
  td_.tracks_[tidx].instances_.clear();
  cout << "[ActiveLearningInterface]  Removed track " << track_id_ << endl;
  reSort();
  
  incrementTrack(-1);
}

void ActiveLearningInterface::labelAs(const Label& annotation)
{
  TrackDataset::Ptr td(new TrackDataset);
  td->applyNameMappings(gc_);
  td->tracks_.push_back(td_->copy(confidences_[track_id_].second));
  td->tracks_[0].setLabel(annotation);

  double imp = 1;
  cout << "[ActiveLearningInterface]  using importance of " << imp << endl;
  td->setImportance(imp);
  
  learner_->pushHandLabeledDataset(td);
  cout << "[ActiveLearningInterface]  labeled as " << annotation.transpose() << ", sent to learner." << endl;

  kickOut();
}

void ActiveLearningInterface::incrementTrack(int num)
{
  scopeLockWrite;

  if(confidences_.empty()) {
    track_id_ = -1;
    return;
  }
  
  track_id_ += num;
  if(track_id_ >= (int)confidences_.size())
    track_id_ = 0;
  if(track_id_ < 0)
    track_id_ = confidences_.size() - 1;

  const Dataset& track = td_[confidences_[track_id_].second];
  if(track.empty())
    frame_id_ = -1;
  else
    frame_id_ = track.size() / 2;
}
  
void ActiveLearningInterface::incrementFrame(int num)
{
  scopeLockWrite;

  if(track_id_ == -1)
    return;

  int tidx = confidences_[track_id_].second;
  const Dataset& track = td_[tidx];
  frame_id_ += num;
  if(frame_id_ >= (int)track.size())
    frame_id_ = 0;
  if(frame_id_ < 0)
    frame_id_ = track.size() - 1;
}

int ActiveLearningInterface::classKeyToIdx(char key) const
{
  return key - 'a';
}

char ActiveLearningInterface::classIdxToKey(int idx) const
{
  return idx + 'a';
}


