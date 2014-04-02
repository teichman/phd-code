#ifndef DISCRETE_BAYES_FILTER_H
#define DISCRETE_BAYES_FILTER_H

#include <online_learning/dataset.h>

class DiscreteBayesFilter
{
public:
  DiscreteBayesFilter(float cap = 10, double weight = 0.1, Label prior = Label());
  void addObservation(Label frame_prediction, const Eigen::VectorXf& centroid, double timestamp);
  Label trackPrediction() const;
  Label mostRecentFramePrediction() const;
  double timestamp() const { return prev_sensor_timestamp_; }
  size_t numObservations() const { return frame_predictions_.size(); }

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

#endif // DISCRETE_BAYES_FILTER_H
