#include <online_learning/projection_slicer.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

namespace odontomachus
{

  /************************************************************
   * LogisticStochasticTrainer
   ************************************************************/

  LogisticStochasticTrainer::LogisticStochasticTrainer() :
    Trainer()
  {
  }

  LogisticStochasticTrainer::~LogisticStochasticTrainer()
  {
    for(size_t i = 0; i < schedulers_.size(); ++i)
      if(schedulers_[i])
	delete schedulers_[i];
  }

  void LogisticStochasticTrainer::reset()
  {
    for(size_t i = 0; i < schedulers_.size(); ++i)
      schedulers_[i]->reset();
  }

  void LogisticStochasticTrainer::setClassifier(Classifier* classifier)
  {
    if(nameMappings().empty())
      applyNameMapping("cmap", classifier->nameMapping("cmap"));
    ROS_ASSERT(classifier->nameMapping("cmap") == nameMapping("cmap"));
    ROS_ASSERT(schedulers_.size() == nameMapping("cmap").size());
    
    slicer_ = dynamic_cast<ProjectionSlicer*>(classifier);
    ROS_ASSERT(slicer_);
    ROS_ASSERT(slicer_->prior_.rows() > 0);
    ROS_FATAL_STREAM_COND(slicer_->projections_.empty(), "Attempted to attach uninitialized projection slicer to LogisticStochasticTrainer.");
  }
  
  void LogisticStochasticTrainer::train(const Dataset& dataset)
  {
    ROS_ASSERT(slicer_);
    ROS_ASSERT(nameMapping("cmap") == dataset.nameMapping("cmap"));
    ROS_ASSERT(slicer_->nameMappingsAreEqual(dataset));
    
    cout << "Scheduler counts: " << endl;
    for(size_t i = 0; i < schedulers_.size(); ++i)
      cout << i << ": " << schedulers_[i]->getNumInstances() << ", learning rate: " << schedulers_[i]->getLearningRate() << endl;
    
    for(int i = 0; i < dataset.size(); ++i) {
      if((int)i % (dataset.descriptors_.cols() / 10) == 0)
	ROS_DEBUG_STREAM("Completed training on " << i << " / " << dataset.descriptors_.cols() << " instances with stochastic logistic regression." << flush);

      train(dataset.descriptors_.col(i), dataset.labels_(i));
    }
  }

  void LogisticStochasticTrainer::train(const Eigen::VectorXf& instance, int label)
  {
    if(label == -2)
      return;
    
    // -- Only train on instances that were classified incorrectly.
    Classification cl = slicer_->classify(instance);
    for(size_t i = 0; i < slicer_->getNumClasses(); ++i) { 
      if((label == (int)i && cl.response_(i) <= 0) ||
	 (label != (int)i && cl.response_(i) >= 0))
      {
	step(instance, cl.response_(i), label, i);
	schedulers_[i]->incrementInstances();
      }
    }
  }

  void LogisticStochasticTrainer::step(const VectorXf& descriptor, double response, int label, int response_class_id)
  {
    double y = -1.0;
    if(label == response_class_id)
      y = 1.0;

    double weight = 1.0 / (1.0 + exp(y * response));
    double update = schedulers_[response_class_id]->getLearningRate() * y * weight;
    slicer_->prior_(response_class_id) += update;

    for(size_t i = 0; i < slicer_->projections_.size(); ++i) {
      Projection& proj = *slicer_->projections_[i];
      int idx = proj.getCellIdx(descriptor(i));
      proj.cells_(response_class_id, idx) += update;
    }
  }

  void LogisticStochasticTrainer::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
  {
    ROS_ASSERT(id == "cmap");
    translator.translate(&schedulers_, (LearningRateScheduler*)NULL);
    for(size_t i = 0; i < schedulers_.size(); ++i)
      if(!schedulers_[i])
	schedulers_[i] = new SqrtScheduler(1);
  }
  
} // namespace odontomachus
