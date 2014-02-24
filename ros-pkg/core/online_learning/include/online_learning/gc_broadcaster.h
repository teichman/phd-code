#ifndef GC_BROADCASTER_H
#define GC_BROADCASTER_H

#include <ros/ros.h>
#include <online_learning/tbssl.h>
#include <blob/blob.h>

// OnlineLearner is designed to be ROS-agnostic.
// GCBroadcaster is for hooking up OnlineLearner to the ROS network.
class GCBroadcaster : public Agent
{
public:
  GCBroadcaster(OnlineLearner* ol);

protected:
  OnlineLearner* ol_;
  ros::NodeHandle nh_;
  ros::Publisher gc_pub_;
  blob::BinaryBlob msg_;
  GridClassifier gc_;

  void _run();
};

#endif // GC_BROADCASTER_H
