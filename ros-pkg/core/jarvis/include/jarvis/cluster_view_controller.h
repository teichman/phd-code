#ifndef CLUSTER_VIEW_CONTROLLER_H
#define CLUSTER_VIEW_CONTROLLER_H

#include <online_learning/tbssl.h>

class ClusterViewController;

// Interface for any class that is used for viewing clusters of tracks.
class ClusterView
{
public:
  virtual ~ClusterView() {}
  // TD contains shared_ptrs.
  virtual void displayCluster(TrackDataset td) = 0;
  virtual void displayMessage(const std::string& message) = 0;
};


// View that specifically displays Blob clusters.
class ClusterBlobView : public Agent, public ClusterView
{
public:
  ClusterBlobView(ClusterViewController* cvc);
  void displayCluster(TrackDataset td);
  void displayMessage(const std::string& message);

protected:
  // Used for sending back keypresses.
  ClusterViewController* cvc_;
  
  void _run();
};

class ClusterViewController : public Agent
{
public:
  ClusterViewController(OnlineLearner* ol);
  void handleKeypress(char key);
  void setView(ClusterView* view) { view_ = view; }
  
protected:
  OnlineLearner* ol_;
  ClusterView* view_;
  
  void _run();
};

#endif // CLUSTER_VIEW_CONTROLLER_H
