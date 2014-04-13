#include <algorithm>

#include <agent/lockable.h>
#include <boost/any.hpp>
#include <jarvis/cluster_view_controller.h>
#include <online_learning/clusterer.h>
#include <jarvis/tracker.h>
#include <online_learning/tbssl.h>
#include <online_learning/dataset.h>

using namespace std;

const static string kClassname = "interesting";  // TODO(hendrik) fixme

void ClusterListVC::removeClusterAndPushToLearner(int index) {
  cout << "Removing cluster #" << index << endl;
  boost::unique_lock<boost::shared_mutex> lock(
      ClusterListView::shared_mutex_);

  TrackDataset::Ptr picked = clusters_[index]->cluster_;
  TrackDataset::Ptr neg_example(new TrackDataset);
  neg_example->applyNameMappings(*picked);

  for (Dataset::Ptr track : picked->tracks_) {
    Label l(track->label());
    l[track->nameMapping("cmap").toId(kClassname)] = -1;
    track->setLabel(l);
    neg_example->tracks_.push_back(track);
  }
  boost::thread pusher(&OnlineLearner::pushHandLabeledDataset, learner_,
                       neg_example);

  clusters_.erase(clusters_.begin() + index);
}

void ClusterListVC::mouse(int button, int state, int x, int y) {
  switch (button) {
  case 0:
  case 3:
  case 4:
    return ClusterListView::mouse(button, state, x ,y);
  case 1:
  case 2:
    if (state == 0) {
      removeClusterAndPushToLearner(indexPicked(x, y));
      PostRedisplay();
    }
  }
}

namespace {
bool score_sort(const Dataset::Ptr& A, const Dataset::Ptr& B) {
  return A->label()[A->nameMapping("cmap").toId(kClassname)]
       < B->label()[B->nameMapping("cmap").toId(kClassname)];
}

bool score_compare(const Dataset::Ptr& A, float val) {
  return A->label()[A->nameMapping("cmap").toId(kClassname)] < val;
}
}

void ClusterListVC::_run() {
  while (true) {
    usleep(1e4);
    if (clustersBeforeEnd() < 2) {
      boost::unique_lock<boost::shared_mutex> lock2(
          Agent::shared_mutex_);

      GridClassifier gc;
      learner_->copyClassifier(&gc);

      TrackDataset td;
      std::vector<double> hashes;
      learner_->viewableUnsupervised(&td, &hashes);
      if (td.size() == 0) {
        usleep(1e5);
        continue;
      }

      sort(td.tracks_.begin(), td.tracks_.end(), score_sort);
      int pos = std::lower_bound(td.tracks_.begin(), td.tracks_.end(), 0.0f,
                                 score_compare)
          - td.tracks_.begin();
      int dir = (pos < (int)td.size()) ? 1 : -1;

      TrackDataset::Ptr clustered(new TrackDataset());
      clustered->applyNameMappings(td);
      {
        boost::shared_lock<boost::shared_mutex> lock(
            ClusterListView::shared_mutex_);

        while (displayed_datasets_.find(td.tracks_[pos]->hash())
            != displayed_datasets_.end())
        {
          pos++;
        }

        clustered->tracks_.push_back(td.tracks_[pos]);
        for (int j = pos + dir; j < (int) td.size() && j >= 0; j += dir)
        {
          if (similar(*clustered->tracks_[0], *td.tracks_[j], gc, 0.7, 3))
          {
            clustered->tracks_.push_back(td.tracks_[j]);
          }
        }
      }
      if(clustered->size() > 0) {
        displayCluster(clustered);
      } else {
        cerr << "end of example list, can't display more" << endl;
        usleep(1e6);
      }
    }
  }
}
