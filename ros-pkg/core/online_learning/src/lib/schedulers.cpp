#include <online_learning/schedulers.h>

using namespace std;

LearningRateScheduler* getScheduler(int scheduler_id)
{
  LearningRateScheduler* scheduler = NULL;
  switch(scheduler_id) {
  case 0:
    scheduler = new ConstantScheduler(1.0);
    break;
  case 2:
    scheduler = new SqrtScheduler(1.0);
    break;
  default:
    ROS_FATAL_STREAM("Requested scheduler id " << scheduler_id << " does not exist.");
  }
  return scheduler;
}

LearningRateScheduler::LearningRateScheduler(double eta_0) :
  eta_0_(eta_0),
  num_instances_(1)
{
}
  
void LearningRateScheduler::incrementInstances()
{
  ++num_instances_;
}

void LearningRateScheduler::serialize(std::ostream& out) const
{
  eigen_extensions::serializeScalar(eta_0_, out);
  eigen_extensions::serializeScalar(num_instances_, out);
}

void LearningRateScheduler::deserialize(std::istream& in)
{
  eigen_extensions::deserializeScalar(in, &eta_0_);
  eigen_extensions::deserializeScalar(in, &num_instances_);
}

double ConstantScheduler::getLearningRate() const
{
  return eta_0_;
}

double SqrtScheduler::getLearningRate() const
{
  return eta_0_ / sqrt(num_instances_);
}
