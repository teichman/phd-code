#ifndef CLUSTER_VIEW_CONTROLLER_H
#define CLUSTER_VIEW_CONTROLLER_H

#include <online_learning/track_dataset_visualizer.h>
#include <jarvis/view_controller.h>

using boost::shared_ptr;

class Dataset;
class GridClassifier;
class OnlineLearner;
class Texture;
class TrackDataset;

// View that displays Blob clusters.
class ClusterBlobView : public ViewController, public ClusterView
{
public:
  typedef shared_ptr<ClusterBlobView> Ptr;
  ClusterBlobView();

  virtual void displayCluster(shared_ptr<TrackDataset> td);
  virtual void displayMessage(const std::string& message);

  virtual void display();

//  virtual void mouse(int button, int state, int x, int y) {}
//  virtual void motion(int x, int y) {}
  virtual void key(unsigned char k, int x, int y);

protected:
  shared_ptr<TrackDataset> td_;
  size_t instance_displayed_;
};


#endif // CLUSTER_VIEW_CONTROLLER_H
