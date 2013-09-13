#include <online_learning/projection_slicer.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

namespace odontomachus
{

  void Classifier::setTrainer(Trainer::Ptr trainer)
  {
    trainer_ = trainer;
    trainer_->setClassifier(this);
  }
  
  /************************************************************
   * Projection
   ************************************************************/
  
  Projection::Projection() :
    min_(FLT_MAX),
    max_(-FLT_MAX),
    width_(0),
    inv_width_(-1)
  {
    
  }

  void Projection::reset()
  {
    cells_.setZero();
  }
  
  void Projection::initialize(size_t num_cells, const Eigen::VectorXf& vals)
  {
    ROS_ASSERT(!nameMapping("cmap").empty());
	
    bool verbose = false;
    if(rand() % 100 == 0)
      verbose = true;

    if(verbose)
      ROS_DEBUG_STREAM("Initializing Projection using " << vals.rows() << " instances.");

    // -- Get the min and max.
    //    TODO: don't be so sensitive to outliers.  Fit a Gaussian, use 3stdevs or something.
    max_ = vals.maxCoeff();
    min_ = vals.minCoeff();
    if(verbose) { 
      ROS_DEBUG_STREAM("Max value during initialization: " << max_);
      ROS_DEBUG_STREAM("Min value during initialization: " << min_);
    }
    
    // -- Choose a bin width.
    width_ = (max_ - min_) / (double)num_cells;
    inv_width_ = 1.0 / width_;
    if(verbose)
      ROS_DEBUG_STREAM("Cell width set to " << width_);
    
    // -- Allocate cells.
    cells_ = MatrixXf::Zero(nameMapping("cmap").size(), num_cells);
  }
  
  void Projection::serialize(std::ostream& out) const
  {
    eigen_extensions::serialize(cells_, out);
    out.write((char*)&min_, sizeof(min_));
    out.write((char*)&max_, sizeof(max_));
    out.write((char*)&width_, sizeof(width_));
  }
  
  void Projection::deserialize(std::istream& in)
  {
    eigen_extensions::deserialize(in, &cells_);
    in.read((char*)&min_, sizeof(min_));
    in.read((char*)&max_, sizeof(max_));
    in.read((char*)&width_, sizeof(width_));
    inv_width_ = 1.0 / width_;
  }
  
  int Projection::getCellIdx(float val) const
  {
    return std::min((int)cells_.cols() - 1, std::max(0, (int)((val - min_) * inv_width_)));
  }

  void Projection::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
  {
    ROS_ASSERT(id == "cmap");
    translator.translateRows(&cells_);
  }

  
  /************************************************************
   * ProjectionSlicer
   ************************************************************/
  
  ProjectionSlicer::ProjectionSlicer()
  {
  }

  void ProjectionSlicer::_reset()
  {
    for(size_t i = 0; i < projections_.size(); ++i)
      projections_[i]->reset();
    prior_.setZero();
  }

  void ProjectionSlicer::initialize(size_t num_cells, const Dataset& dataset)
  {
    ROS_ASSERT(nameMappings().empty());
    applyNameMappings(dataset);
    ROS_ASSERT(nameMappingsAreEqual(dataset));
    ROS_ASSERT((int)projections_.size() == dataset.descriptors_.rows());
    
    for(size_t i = 0; i < projections_.size(); ++i)
      projections_[i]->initialize(num_cells, dataset.descriptors_.row(i));
  }

  Classification ProjectionSlicer::classify(const Eigen::VectorXf& instance) const
  {
    ROS_ASSERT((size_t)instance.rows() == projections_.size());
    
    Classification cl;
    cl.response_ = prior_;

    for(size_t i = 0; i < projections_.size(); ++i) {
      int idx = projections_[i]->getCellIdx(instance(i));
      for(int j = 0; j < cl.response_.rows(); ++j) 
	cl.response_.coeffRef(j) += projections_[i]->cells_.coeffRef(j, idx);
    }

    return cl;
  }
    
  size_t ProjectionSlicer::getNumCells() const
  {
    ROS_ASSERT(!projections_.empty());
    return projections_[0]->cells_.cols();
  }

  size_t ProjectionSlicer::getNumClasses() const
  {
    return prior_.rows();
  }

  size_t ProjectionSlicer::getNumParamsPerClass() const
  {
    int num_params = 1; // prior
    for(size_t i = 0; i < projections_.size(); ++i)
      num_params += projections_[i]->cells_.cols();
    return num_params;
  }
  
  void ProjectionSlicer::serialize(std::ostream& out) const
  {
    serializeNameMappings(out);
    for(size_t i = 0; i < projections_.size(); ++i)
      projections_[i]->serialize(out);
    eigen_extensions::serialize(prior_, out);
  }
  
  void ProjectionSlicer::deserialize(std::istream& in)
  {
    deserializeNameMappings(in);
    ROS_ASSERT(!projections_.empty());
    for(size_t i = 0; i < projections_.size(); ++i) {
      projections_[i] = Projection::Ptr(new Projection());
      projections_[i]->deserialize(in);
    }
    eigen_extensions::deserialize(in, &prior_);
  }

  void ProjectionSlicer::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
  {    
    if(id == "cmap") {
      translator.translate(&prior_);
      for(size_t i = 0; i < projections_.size(); ++i)
	projections_[i]->applyNameTranslator(id, translator);

      if(trainer_)
	trainer_->applyNameTranslator(id, translator);
    }
    else if(id == "dmap") {
      translator.translate(&projections_, Projection::Ptr());

      // New projections are initialized with default ctor, which is a NULL
      // boost::shared_ptr.  We need to initialize these.
      for(size_t i = 0; i < projections_.size(); ++i) {
	if(!projections_[i]) {
	  projections_[i] = Projection::Ptr(new Projection());
	  projections_[i]->applyNameMapping("cmap", nameMapping("cmap"));
	}
      }
    }
    else
      ROS_ASSERT(0);
  }
  
  /************************************************************
   * VanillaSLR
   ************************************************************/

  VanillaSLR::VanillaSLR()
  {
  }

  VanillaSLR::~VanillaSLR()
  {
    for(size_t i = 0; i < schedulers_.size(); ++i)
      if(schedulers_[i])
	delete schedulers_[i];
  }
  
  Classification VanillaSLR::classify(const Eigen::VectorXf& instance) const
  {
    Classification cl;
    cl.response_ = prior_ + weights_ * instance;
    return cl;
  }

  void VanillaSLR::train(const Dataset& dataset)
  {
    ROS_ASSERT(nameMappingsAreEqual(dataset));
    
    for(int i = 0; i < dataset.size(); ++i) {
      if((int)i % (dataset.descriptors_.cols() / 10) == 0)
	ROS_DEBUG_STREAM("Completed training on " << i << " / " << dataset.descriptors_.cols() << " instances with stochastic logistic regression." << flush);

      if(dataset.labels_(i) == -2)
	continue;

      const VectorXf& desc = dataset.descriptors_.col(i);
      const VectorXf& response = classify(desc).response_;
      for(int j = 0; j < weights_.rows(); ++j) {
	double y = -1.0;
	if(dataset.labels_(i) == (int)j)
	  y = 1.0;

	step(desc, response(j), y, j);
	schedulers_[j]->incrementInstances();
      }
    }
  }

  void VanillaSLR::step(const VectorXf& descriptor, double response, double y, int response_class_id)
  {
    double weight = 1.0 / (1.0 + exp(y * response));
    double update = schedulers_[response_class_id]->getLearningRate() * y * weight;
    prior_(response_class_id) += update;
    weights_.row(response_class_id) += update * descriptor.transpose();
  }

  void VanillaSLR::serialize(std::ostream& out) const
  {
    cout << "Not serializing VanillaSLR." << endl;
  }

  void VanillaSLR::deserialize(std::istream& in)
  {
    ROS_ASSERT(0);
  }

  void VanillaSLR::_reset()
  {
    for(size_t i = 0; i < schedulers_.size(); ++i) 
      schedulers_[i]->reset();
    
    weights_.setZero();
    prior_.setZero();
    ROS_ASSERT(!trainer_);
  }

  void VanillaSLR::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
  {
    if(id == "cmap") {
      translator.translate(&schedulers_, (LearningRateScheduler*)NULL);
      for(size_t i = 0; i < schedulers_.size(); ++i)
	if(!schedulers_[i])
	  schedulers_[i] = new SqrtScheduler(1);
    
      translator.translateRows(&weights_);
      translator.translate(&prior_);
    }
    else if(id == "dmap")
      translator.translateCols(&weights_);
    else
      ROS_ASSERT(0);

    // -- If we just got both the dmap and cmap, then allocate weights_.
    if(weights_.rows() == 0 && weights_.cols() == 0 &&
       hasNameMapping("dmap") && !nameMapping("dmap").empty() &&
       hasNameMapping("dmap") && !nameMapping("cmap").empty())
    {
      weights_ = MatrixXf::Zero(nameMapping("cmap").size(), nameMapping("dmap").size());
    }
  }
  
} // namespace
