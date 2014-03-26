#include <jarvis/discrete_bayes_filter.h>

using namespace std;
using namespace Eigen;

DiscreteBayesFilter::DiscreteBayesFilter(float cap, double weight, Label prior) :
  cap_(cap),
  weight_(weight),
  prior_(prior)
{
}

void DiscreteBayesFilter::addObservation(Label frame_prediction,
                                         const Eigen::VectorXf& centroid,
                                         double timestamp)
{
  frame_predictions_.push_back(frame_prediction);
  
  if(frame_predictions_.size() == 1)
    cumulative_ = frame_prediction;
  else {
    double velocity = (centroid - prev_centroid_).norm() / (timestamp - prev_sensor_timestamp_);
    cumulative_ += frame_prediction * velocity * weight_;
  }

  for(int i = 0; i < cumulative_.rows(); ++i)
    cumulative_[i] = max(-cap_, min(cap_, cumulative_[i]));

  prev_centroid_ = centroid;
  prev_sensor_timestamp_ = timestamp;

  if(prior_.rows() == 0)
    prior_ = VectorXf::Zero(cumulative_.rows());
}

Label DiscreteBayesFilter::trackPrediction() const
{
  // TODO: Label needs to be fixed so that this can be one line.
  Label track_prediction = cumulative_;
  track_prediction += prior_;
  return track_prediction;
}


Label DiscreteBayesFilter::mostRecentFramePrediction() const
{
  if(frame_predictions_.empty())
    return prior_;

  return frame_predictions_.back();
}
