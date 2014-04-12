#ifndef CLUSTER_VIEW_CONTROLLER_H
#define CLUSTER_VIEW_CONTROLLER_H

#include <vector>

//#include <online_learning/track_dataset_visualizer.h>
#include <agent/agent.h>
#include <jarvis/view_controller.h>

using boost::shared_ptr;

class Dataset;
class GridClassifier;
class Texture;
class TrackDataset;
class OnlineLearner;

class ClusterView
{
public:
  typedef shared_ptr<ClusterView> Ptr;
  ClusterView(boost::shared_ptr<TrackDataset> cluster);
  virtual ~ClusterView() {}

  // renders the cluster into unit rectangle
  virtual void display();

protected:
  shared_ptr<TrackDataset> cluster_;
  size_t instance_displayed_, max_instances_;
};


class ClusterListView : public SharedLockable, public ViewController {
public:
  ClusterListView()
      : scroll_position_(0), scroll_momentum_(0), dragging_(false) {}
  virtual ~ClusterListView() {}

  virtual void display();
  virtual void motion(int x, int y);
  virtual void mouse(int button, int button_state, int x, int y);

  void clear();
  // number of clusters user can scroll down before reaching the end of list
  // can be <=0 if clusters are urgently needed
  int clustersBeforeEnd();
  virtual void displayCluster(boost::shared_ptr<TrackDataset> cluster);

  // returns index into clusters_ that user clicked on (-1 for none)
  int indexPicked(int mouse_x, int mouse_y);

protected:
  std::vector<ClusterView::Ptr> clusters_;
  float scroll_position_;
  float scroll_momentum_;  // clusters/sec
  double scroll_last_timestep_;
  bool dragging_;
  int last_mouse_y_;
};


class ClusterListVC : public ClusterListView, public Agent {
public:
  typedef shared_ptr<ClusterListVC> Ptr;
  ClusterListVC() : learner_(NULL) {}
  virtual ~ClusterListVC() {}

  void setOnlineLearner(OnlineLearner* learner) { learner_ = learner; }

  void addAllSimilarTo(boost::shared_ptr<Dataset> reference,
                       boost::shared_ptr<TrackDataset> td,
                       const GridClassifier &gc);
  virtual void mouse(int button, int state, int x, int y);

  void _run();

protected:
  OnlineLearner* learner_;
};

#endif // CLUSTER_VIEW_CONTROLLER_H
