#ifndef PROJECTION_SLICER_H
#define PROJECTION_SLICER_H

#include <float.h>
#include <boost/thread.hpp>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <optimization/optimization.h>
#include <optimization/common_functions.h>
#include <logistic/logistic.h>
#include <bag_of_tricks/high_res_timer.h>
#include <online_learning/online_learning.h>
#include <online_learning/schedulers.h>

namespace odontomachus { 

  class VanillaSLR : public Classifier
  {
  public:
    typedef boost::shared_ptr<VanillaSLR> Ptr;
    typedef boost::shared_ptr<const VanillaSLR> ConstPtr;
    
    std::vector<LearningRateScheduler*> schedulers_;
    //! class x descriptor weight.
    Eigen::MatrixXf weights_;
    //! Training descriptors should not include the dummy 1.
    Eigen::VectorXf prior_;
    
    VanillaSLR();
    ~VanillaSLR();
    Classification classify(const Eigen::VectorXf& instance) const;
    void train(const Dataset& dataset);
    void step(const Eigen::VectorXf& descriptor, double response, double y, int response_class_id);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    Eigen::VectorXf getPrior() const { return prior_; }

    void _applyNameTranslator(const std::string& id, const NameTranslator& translator);

  protected:
    void _reset();
  };

  class Projection : public Serializable, public NameMappable
  {
  public:
    typedef boost::shared_ptr<Projection> Ptr;
    typedef boost::shared_ptr<const Projection> ConstPtr;

    //! cells_(i, j) is the response for class i, cell j.
    Eigen::MatrixXf cells_;
    
    Projection();
    //! Sets the bounds and allocates bins based on a sample of data.
    void initialize(size_t num_cells, const Eigen::VectorXf& vals);
    //Eigen::VectorXf classify(float val) const;
    int getCellIdx(float val) const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    void reset();
    
  protected:
    double min_;
    double max_;
    double width_;
    double inv_width_;

    void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
  };
  
  class ProjectionSlicer : public Classifier
  {
  public:
    typedef boost::shared_ptr<ProjectionSlicer> Ptr;
    typedef boost::shared_ptr<const ProjectionSlicer> ConstPtr;
     
    std::vector<Projection::Ptr> projections_;
    Eigen::VectorXf prior_;
    
    ProjectionSlicer();
    //! Initializes all weak classifiers.  Doesn't need to be labeled.
    void initialize(size_t num_cells, const Dataset& dataset);
    Classification classify(const Eigen::VectorXf& instance) const;
    size_t getNumCells() const;
    size_t getNumClasses() const;
    //! Returns numbers of cells in all projections, plus one for the prior.
    //! Does not include the projection_weights_.
    size_t getNumParamsPerClass() const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    Eigen::VectorXf getPrior() const { return prior_; }
    int getDimensionality() const { return projections_.size(); }

  protected:
    // No copying.
    ProjectionSlicer(const ProjectionSlicer& other);
    ProjectionSlicer& operator=(const ProjectionSlicer& other);

    void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
    void _reset();
  };

  class LogisticStochasticTrainer : public Trainer
  {
  public:
    typedef boost::shared_ptr<LogisticStochasticTrainer> Ptr;
    typedef boost::shared_ptr<const LogisticStochasticTrainer> ConstPtr;

    //! One per class problem.  Automatically set to sqrt.
    std::vector<LearningRateScheduler*> schedulers_;

    LogisticStochasticTrainer();
    ~LogisticStochasticTrainer();

  protected:
    ProjectionSlicer* slicer_;

    void train(const Eigen::VectorXf& instance, int label);
    //! Takes a gradient step for each training instance in the dataset.
    //! Uses a random order.
    void train(const Dataset& dataset);
    void step(const Eigen::VectorXf& descriptor, double response, int label, int response_class_id);
    void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
    void setClassifier(Classifier* classifier);
    void reset();
  };
  
} // namespace

#endif // PROJECTION_SLICER_H
