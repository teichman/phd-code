#ifndef LEARNING_RATE_SCHEDULERS_H
#define LEARNING_RATE_SCHEDULERS_H

#include <math.h>
#include <stddef.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>

class LearningRateScheduler : public Serializable
{
public:
  LearningRateScheduler(double eta_0);
  virtual ~LearningRateScheduler() {};
  virtual double getLearningRate() const = 0;
  void incrementInstances();
  void reset() { num_instances_ = 1; }
  double getNumInstances() const { return num_instances_; }
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  
protected:
  double eta_0_;
  double num_instances_;
};

class ConstantScheduler : public LearningRateScheduler
{
public:
  ConstantScheduler(double eta_0)  : LearningRateScheduler(eta_0) {}
  double getLearningRate() const;
};

class SqrtScheduler : public LearningRateScheduler
{
public:
  SqrtScheduler(double eta_0)  : LearningRateScheduler(eta_0) {}
  double getLearningRate() const;
};

LearningRateScheduler* getScheduler(int scheduler_id);

#endif // LEARNING_RATE_SCHEDULERS_H
