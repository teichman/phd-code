#include <online_learning/evaluator.h>
#include <boost/filesystem.hpp>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

using namespace std;
using namespace Eigen;
namespace fs = boost::filesystem;


Evaluator::Evaluator(Classifier::ConstPtr classifier) :
  classifier_(classifier),
  frame_stats_(classifier->nameMapping("cmap")),
  track_stats_(classifier->nameMapping("cmap")),
  evaluation_timer_("Total evaluation time"),
  plot_(true)
{
}
  
void Evaluator::evaluate(const TrackDataset& test)
{
  evaluation_timer_.start();
      
  ROS_ASSERT(classifier_);
  ROS_ASSERT(classifier_->nameMappingsAreEqual(test));

  ROS_WARN_ONCE("Evaluator is based on PerfStats which does not handle {-1, 0, +1}^c labels properly.");
  for(size_t i = 0; i < test.size(); ++i) {
    int id = test[i][0].label_.id();
    if(id == -2)
      continue;

    Label tpred = VectorXf::Zero(classifier_->prior().rows());
    for(size_t j = 0; j < test[i].size(); ++j) { 
      Label fpred = classifier_->classify(test[i][j]);
      frame_stats_.incrementStats(id, fpred);
      tpred += fpred - classifier_->prior();
    }
    tpred /= (double)test[i].size();
    tpred += classifier_->prior();
    track_stats_.incrementStats(id, tpred);
  }

  evaluation_timer_.stop();
}	
   
void Evaluator::evaluateParallel(const TrackDataset& test, Eigen::MatrixXf* annotations_ptr, Eigen::MatrixXf* predictions_ptr)
{
  evaluation_timer_.start();
    
  ROS_ASSERT(classifier_);
  ROS_ASSERT(classifier_->nameMappingsAreEqual(test));

  MatrixXf annotations(test.size(), classifier_->nameMapping("cmap").size());
  MatrixXf predictions(test.size(), classifier_->nameMapping("cmap").size());
  annotations.setZero();
  predictions.setZero();

  // -- Build an index for efficient parallel computation.
  vector<size_t> tid;
  vector<size_t> fid;
  tid.reserve(test.totalInstances());
  fid.reserve(test.totalInstances());
  for(size_t i = 0; i < test.size(); ++i) {
    int id = test[i][0].label_.id();
    if(id == -2)
      continue;
    for(size_t j = 0; j < test[i].size(); ++j) {
      tid.push_back(i);
      fid.push_back(j);
    }
  }

  // -- Classify all frames.
  vector<Label> fpreds(test.totalInstances());
  HighResTimer hrt("classification");
  hrt.start();
  #pragma omp parallel for
  for(size_t i = 0; i < tid.size(); ++i) {
    // ROS_ASSERT(tid[i] <= test.size());
    // ROS_ASSERT(fid[i] <= test[tid[i]].size());
    fpreds[i] = classifier_->classify(test[tid[i]][fid[i]]);
  }
  hrt.stop();
  ROS_DEBUG_STREAM("Evaluator: Classification of " << tid.size() << " instances complete in " << hrt.getSeconds() << " seconds.  ~ "
		   << hrt.getSeconds() / (double)tid.size() << " seconds per instance." << flush);
  
  // -- Accumulate predictions.
  size_t idx = 0;
  for(size_t i = 0; i < test.size(); ++i) {
    int id = test[i][0].label_.id();
    if(id == -2)
      continue;
      
    Label tpred = VectorXf::Zero(classifier_->prior().rows());
    for(size_t j = 0; j < test[i].size(); ++j, ++idx) { 
      const Label& fpred = fpreds[idx];
      frame_stats_.incrementStats(id, fpred);
      tpred += fpred - classifier_->prior();
    }
    tpred /= (double)test[i].size();
    tpred += classifier_->prior();
    track_stats_.incrementStats(id, tpred);

    annotations.row(i) = test[i][0].label_.transpose();
    predictions.row(i) = tpred.transpose();
  }

  if(annotations_ptr)
    *annotations_ptr = annotations;
  if(predictions_ptr)
    *predictions_ptr = predictions;

  evaluation_timer_.stop();
}
  
void Evaluator::saveResults(const std::string& path)
{
  ROS_ASSERT(fs::is_directory(path));

  frame_stats_.save(path + "/frame_results.txt");
  if(plot_) {
    frame_stats_.saveConfusionMatrix(path + "/frame_confusion_matrix.png");
    frame_stats_.saveConfusionMatrix(path + "/frame_confusion_matrix.pdf");
    frame_stats_.savePrecisionRecallCurve(path + "/frame_pr.png");
    frame_stats_.savePrecisionRecallCurve(path + "/frame_pr.pdf");
  }

  track_stats_.save(path + "/track_results.txt");
  if(plot_) {
    track_stats_.saveConfusionMatrix(path + "/track_confusion_matrix.png");
    track_stats_.saveConfusionMatrix(path + "/track_confusion_matrix.pdf");
    track_stats_.savePrecisionRecallCurve(path + "/track_pr.png");
    track_stats_.savePrecisionRecallCurve(path + "/track_pr.pdf");
  }
}
