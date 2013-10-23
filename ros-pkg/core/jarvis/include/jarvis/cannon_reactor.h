#ifndef REACTOR_CANNON_H
#define REACTOR_CANNON_H

#include <timer/timer.h>
#include <online_learning/dataset.h>
#include <jarvis/reactor.h>
#include <bag_of_tricks/agent.h>

class CannonDriver : public Agent
{
public:
  CannonDriver();
  void fire() { firing_ = true; }
  bool firing() const { return firing_; }
  void _run();
  
protected:
  bool firing_;
  int num_darts_;

  void sendFireMessage();
};

class DiscreteBayesFilter
{
public:
  DiscreteBayesFilter(float cap = 20, double weight = 10, Label prior = Label());
  void addObservation(jarvis::DetectionConstPtr msg);
  Label trackPrediction() const;
  double timestamp() const { return prev_sensor_timestamp_; }

protected:
  float cap_;
  //! Each log odds vector is weighted by dcentroid * weight.
  //! i.e. if the thing isn't moving, assume that we're not getting new information.
  double weight_;
  Label prior_;
  std::vector<Label> frame_predictions_;
  Label cumulative_;
  Eigen::VectorXf prev_centroid_;
  double prev_sensor_timestamp_;
};

class CannonReactor : public Reactor
{
public:
  CannonReactor(double threshold = 10.0);
  ~CannonReactor();
  
protected:
  double threshold_;
  HighResTimer hrt_;
  CannonDriver cannon_driver_;
  std::map<size_t, DiscreteBayesFilter> filters_;
  
  void detectionCallback(jarvis::DetectionConstPtr msg);
};

#endif // REACTOR_CANNON_H
