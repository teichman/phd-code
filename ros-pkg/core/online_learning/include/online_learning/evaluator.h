#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <performance_statistics/performance_statistics.h>
#include <bag_of_tricks/high_res_timer.h>
#include <online_learning/classifier.h>

class Evaluator
{
public:
  Classifier::ConstPtr classifier_;
  PerfStats frame_stats_;
  PerfStats track_stats_;
  HighResTimer evaluation_timer_;
  bool plot_;

  Evaluator(Classifier::ConstPtr classifier);
  void evaluate(const TrackDataset& test);
  void evaluateParallel(const TrackDataset& test,
			Eigen::MatrixXf* annotations_ptr = NULL,
			Eigen::MatrixXf* predictions_ptr = NULL);
  //! path must be a directory.
  void saveResults(const std::string& path);
};

#endif // EVALUATOR_H
