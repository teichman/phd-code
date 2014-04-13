#ifndef CLUSTER_VIEW_CONTROLLER_H
#define CLUSTER_VIEW_CONTROLLER_H

#include <vector>
#include <set>

//#include <online_learning/track_dataset_visualizer.h>
#include <agent/agent.h>
#include <jarvis/trackball.h>

using boost::shared_ptr;

class Dataset;
class GridClassifier;
class Texture;
class TrackDataset;
class OnlineLearner;
class HighResTimer;

class ClusterView
{
public:
  typedef shared_ptr<ClusterView> Ptr;
  ClusterView(shared_ptr<TrackDataset> cluster);
  virtual ~ClusterView() {}

  // renders the cluster into unit rectangle
  virtual void display();

  shared_ptr<TrackDataset> cluster_;
protected:
  boost::shared_ptr<Texture> tex_;
  std::vector<uint8_t> img_;
  size_t instance_displayed_, max_instances_;
};


class ClusterListView : public SharedLockable, public Trackball {
public:
  ClusterListView();
  virtual ~ClusterListView() {}

  virtual void display();
  void applyScrollMotion(float scroll_motion);
  void applyScrollMomentum();
  virtual void motion(int x, int y);
  virtual void mouse(int button, int button_state, int x, int y);

  void clear();
  // number of clusters user can scroll down before reaching the end of list
  // can be <=0 if clusters are urgently needed
  int clustersBeforeEnd();
  virtual void displayCluster(shared_ptr<TrackDataset> cluster);

  // returns index into clusters_ that user clicked on (-1 for none)
  int indexPicked(int mouse_x, int mouse_y);

protected:
  std::vector<ClusterView::Ptr> clusters_;
  float scroll_position_;
  float scroll_momentum_;  // clusters/sec
  shared_ptr<HighResTimer> scroll_last_update_;
  bool dragging_;
  int last_mouse_y_;
  std::set<double> displayed_datasets_;
};


class ClusterListVC : public ClusterListView, public Agent {
public:
  typedef shared_ptr<ClusterListVC> Ptr;
  ClusterListVC() : learner_(NULL) {}
  virtual ~ClusterListVC() {}

  void setOnlineLearner(OnlineLearner* learner) { learner_ = learner; }

  void addAllClustersIn(shared_ptr<TrackDataset> td,
                        shared_ptr<GridClassifier> gc);
  virtual void mouse(int button, int state, int x, int y);

  void _run();

protected:
  OnlineLearner* learner_;
//  shared_ptr<TrackDataset> all_data_;
//  size_t all_data_pos_;
//  shared_ptr<GridClassifier> gc_;

  void removeClusterAndPushToLearner(int index);
};

#endif // CLUSTER_VIEW_CONTROLLER_H
