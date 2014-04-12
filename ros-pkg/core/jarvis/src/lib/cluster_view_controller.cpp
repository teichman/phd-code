#include <algorithm>

#include <agent/lockable.h>
#include <boost/any.hpp>
#include <jarvis/cluster_view_controller.h>
#include <online_learning/clusterer.h>
#include <jarvis/tracker.h>
#include <online_learning/tbssl.h>
#include <online_learning/dataset.h>

using namespace std;

void ClusterListVC::mouse(int button, int state, int x, int y) {
  if (button == 0) {
    return ClusterListView::mouse(button, state, x ,y);
  } else {
    boost::unique_lock<boost::shared_mutex> lock(ClusterListView::shared_mutex_);
  }
}

void ClusterListVC::_run() {
  while (true) {
    usleep(1e6);
  }
}

void ClusterListVC::addAllSimilarTo(boost::shared_ptr<Dataset> reference,
                                    boost::shared_ptr<TrackDataset> td,
                                    const GridClassifier &gc) {
  TrackDataset::Ptr clustered_(new TrackDataset());
  for(size_t i = 0; i < td->size(); i++) {
    ROS_ASSERT(td->tracks_[i]);
    if(similar(*reference, *td->tracks_[i], gc, 0.7, 3)) {
      clustered_->tracks_.push_back(td->tracks_[i]);
    }
  }
  cout << "Clustered " << clustered_->size() << " tracks." << endl;
  displayCluster(clustered_);
}

