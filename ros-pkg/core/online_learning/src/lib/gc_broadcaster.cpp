#include <online_learning/gc_broadcaster.h>

using namespace std;

GCBroadcaster::GCBroadcaster(OnlineLearner* ol) :
  ol_(ol)
{
  gc_pub_ = nh_.advertise<blob::BinaryBlob>("grid_classifier", 0);  
}

void GCBroadcaster::_run()
{
  while(!quitting_) {
    usleep(5e6);

    GridClassifier gc;
    ol_->copyClassifier(&gc);
    if(gc != gc_) {
      gc_ = gc;
      blob::toBinaryBlob(gc_, &msg_);
      gc_pub_.publish(msg_);
    }
  }
}
