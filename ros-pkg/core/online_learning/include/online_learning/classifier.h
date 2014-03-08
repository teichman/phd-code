#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <timer/timer.h>
#include <online_learning/dataset.h>
//#include <online_learning/schedulers.h>

class Indices : public std::vector<int>
{
public:
  typedef boost::shared_ptr<Indices> Ptr;
  typedef boost::shared_ptr<const Indices> ConstPtr;

  //! empty
  Indices() {}
  //! 0, 1, ..., num.
  static Indices All(size_t num);
};

//! Classifier objects are containers for the model, plus
//! methods for making predictions with that model.
class Classifier : public Serializable, public NameMappable
{
public:
  // If you used DerivedClassifier::Ptr, make sure you have made
  // the corresponding typedefs in the derived class.
  // Otherwise gcc will be very confused.
  typedef boost::shared_ptr<Classifier> Ptr;
  typedef boost::shared_ptr<const Classifier> ConstPtr;
    
  Label classifyTrack(const Dataset& track) const;
  virtual ~Classifier() {}
  virtual Label classify(const Instance& instance) const = 0;
  virtual Label prior() const = 0;
};

//! Trainer objects have an associated Classifier which they
//! set the parameters of using training data.
class Trainer : public Serializable, public NameMappable
{
public:
  typedef boost::shared_ptr<Trainer> Ptr;
  typedef boost::shared_ptr<const Trainer> ConstPtr;

  Classifier* classifier_;
  
  virtual ~Trainer() {};
  //! Trains on the instances in a random order.
  //! You'll need to have "using Trainer::train" in your derived class
  //! to have access to these.
  virtual void train(const TrackDataset& dataset);
  //! Trains in a random order across all instances in the dataset.
  virtual void train(TrackDataset::ConstPtr dataset);
    //! Trains in a random order across all instances in all datasets.
  virtual void train(const std::vector<TrackDataset::ConstPtr>& datasets);
  //! Trains in a random order on just the tracks listed in indices.
  virtual void train(const std::vector<TrackDataset::ConstPtr>& datasets,
		     const std::vector<Indices>& indices) = 0;
  //! Trains in a random order on just the tracks listed in indices.
  virtual void trainSerial(const std::vector<TrackDataset::ConstPtr>& datasets,
			   std::vector<Indices> indices);
  //! Trains on the instances in a random order.
  virtual void train(const Dataset& dataset);
  //! All Trainers should interpret the label_(c) value as +1 / -1 label
  //! combined with a training example weight, i.e. label_(c) == 5.5 means
  //! that this is a positive example with a weight of 5.5.
  virtual void train(const Instance& instance) = 0;
  //! All Trainers should interpret the label_(c) value as +1 / -1 label
  //! combined with a training example weight, i.e. label_(c) == 5.5 means
  //! that this is a positive example with a weight of 5.5.
  //! This method must be thread-safe, as it is used in the
  //! parallelized minibatch training call.
  virtual void train(const Instance& instance, const Label& prediction) = 0;
};


// TODO: Remove this.
class Index
{
public:
  int dataset_;
  int track_;
  int frame_;
  Index() : dataset_(-1), track_(-1), frame_(-1) {}
  Index(int d, int t, int f) : dataset_(d), track_(t), frame_(f) {}
};

#endif // CLASSIFIER_H
