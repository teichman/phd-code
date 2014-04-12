#include <algorithm>

#include <agent/lockable.h>
#include <boost/any.hpp>
#include <jarvis/cluster_view_controller.h>
#include <online_learning/clusterer.h>
#include <jarvis/tracker.h>
#include <online_learning/tbssl.h>
#include <online_learning/dataset.h>

using namespace std;

const static string kClassname = "cat";  // TODO(hendrik) fixme

void ClusterListVC::mouse(int button, int state, int x, int y) {
  if (button == 0 || button == 3 || button == 4) {
    return ClusterListView::mouse(button, state, x ,y);
  } else {
    if (state == 0) {
      int picked = indexPicked(x, y);
      cout << " picked element " << picked << endl;
      boost::unique_lock<boost::shared_mutex> lock(
          ClusterListView::shared_mutex_);
      clusters_.erase(clusters_.begin() + picked);
      PostRedisplay();
    }
  }
}
bool myfunction (int i,int j) { return (i<j); }



void ClusterListVC::_run() {
  while (true) {
    usleep(1e4);
    if (clustersBeforeEnd() < 2) {
      int to_load = 2 - clustersBeforeEnd();
      cout << "to load: " << to_load << endl;

      boost::shared_ptr<TrackDataset> td(new TrackDataset());

      std::vector<double>* hashes;
      learner_->viewableUnsupervised(td.get(), hashes);

      sort(td->tracks_.begin(), td->tracks_.end(), Comp)

//      TrackDataset::Ptr clustered(new TrackDataset());
//      {
//        boost::shared_lock<boost::shared_mutex> lock(
//            ClusterListView::shared_mutex_);
//        boost::unique_lock<boost::shared_mutex> lock2(
//            Agent::shared_mutex_);
//
//        while (displayed_datasets_.find(all_data_->tracks_[all_data_pos_]->hash())
//              != displayed_datasets_.end()) {
//          cerr << " already displayed: " << all_data_pos_;
//          all_data_pos_++;
//        }
//        clustered->tracks_.push_back(all_data_->tracks_[all_data_pos_]);
//        for(size_t j = all_data_pos_+1; j < all_data_->size(); j++) {
//          if(similar(*clustered->tracks_[0], *all_data_->tracks_[j], *gc_, 0.7, 3)) {
//            clustered->tracks_.push_back(all_data_->tracks_[j]);
//          }
//        }
//      }
//      displayCluster(clustered);
    }
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

void ClusterListVC::addAllClustersIn(boost::shared_ptr<TrackDataset> td,
                                     boost::shared_ptr<GridClassifier> gc) {
  boost::unique_lock<boost::shared_mutex> lock(
      Agent::shared_mutex_);
  all_data_ = td;
  all_data_pos_ = 0;
  gc_ = gc;
}

