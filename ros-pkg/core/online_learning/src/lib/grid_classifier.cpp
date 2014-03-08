#include <float.h>
#include <eigen_extensions/eigen_extensions.h>
#include <eigen_extensions/random.h>
#include <online_learning/grid_classifier.h>
#include <gperftools/profiler.h>
#include <online_learning/schedulers.h>

using namespace std;
using namespace Eigen;

#define ETA 0.01
#define TBSSL_EXP exp
//#define TBSSL_EXP fastexp

std::ostream& operator<<(std::ostream& out, const GridClassifier::CellIndex& ci)
{
  out << "[" << ci.ridx_ << ", " << ci.didx_ << ", " << ci.eidx_ << ", " << ci.cidx_ << "]";
  return out;
}

GridClassifier::GridClassifier()
{
}

GridClassifier& GridClassifier::operator=(const GridClassifier& other)
{
  if(&other == this)
    return *this;

  applyNameMappings(other);
  prior_ = other.prior_;
  num_cells_ = other.num_cells_;

  // -- Make new copies of all the grids.
  grids_ = other.grids_;  // Just to get the right sizes easily.
  for(size_t i = 0; i < grids_.size(); ++i) {
    for(size_t j = 0; j < grids_[i].size(); ++j) {
      for(size_t k = 0; k < grids_[i][j].size(); ++k) {
	ROS_ASSERT(grids_[i][j][k]);
	grids_[i][j][k] = new Grid(*grids_[i][j][k]);
      }
    }
  }
  
  return *this;
}

GridClassifier::GridClassifier(const GridClassifier& other) :
  SharedLockable()
{
  *this = other;
}

GridClassifier::~GridClassifier()
{
  for(size_t i = 0; i < grids_.size(); ++i)
    for(size_t j = 0; j < grids_[i].size(); ++j)
      for(size_t k = 0; k < grids_[i][j].size(); ++k)
	if(grids_[i][j][k])
	  delete grids_[i][j][k];
}

bool GridClassifier::operator==(const GridClassifier& other)
{
  if(!nameMappingsAreEqual(other))
    return false;

  if(prior_.rows() != other.prior_.rows())
    return false;
  for(int i = 0; i < prior_.rows(); ++i)
    if(prior_(i) != other.prior_(i))
      return false;

  if(num_cells_.size() != other.num_cells_.size())
    return false;
  for(size_t i = 0; i < num_cells_.size(); ++i)
    if(num_cells_[i] != other.num_cells_[i])
      return false;
  
  if(grids_.size() != other.grids_.size())
    return false;
  for(size_t i = 0; i < grids_.size(); ++i) {
    if(grids_[i].size() != other.grids_[i].size())
      return false;
    for(size_t j = 0; j < grids_[i].size(); ++j) {
      if(grids_[i][j].size() != other.grids_[i][j].size())
	return false;
      for(size_t k = 0; k < grids_[i][j].size(); ++k) {
	ROS_ASSERT(grids_[i][j][k]);
	ROS_ASSERT(other.grids_[i][j][k]);
	ROS_ASSERT(grids_[i][j][k] != other.grids_[i][j][k]);
	if(*grids_[i][j][k] != *other.grids_[i][j][k])
	  return false;
      }
    }
  }

  return true;
}

void GridClassifier::initialize(const TrackDataset& dataset, size_t num_cells)
{
  vector<size_t> vec(1, num_cells);
  initialize(dataset, vec);
}

void GridClassifier::initialize(const TrackDataset& dataset, std::vector<size_t> num_cells)
{ 
  grids_.resize(num_cells.size());
  
  ROS_ASSERT(nameMappings().empty());
  applyNameMappings(dataset);
  ROS_ASSERT(nameMappingsAreEqual(dataset));
  for(size_t i = 0; i < grids_.size(); ++i) {
    ROS_ASSERT(grids_[i].size() == dataset.nameMapping("dmap").size());
    for(size_t j = 0; j < grids_[i].size(); ++j)
      ROS_ASSERT(grids_[i][j].empty());
  }
  
  num_cells_ = num_cells;
  reinitialize(dataset);
}

void GridClassifier::reinitialize(const TrackDataset& dataset)
{
  ROS_ASSERT(nameMappingsAreEqual(dataset));
  ROS_ASSERT(dataset.size() > 0);

  // All descriptor spaces must have at least one example which
  // determines the dimensionality.
  // This is a pretty extreme edge case.
  DescriptorDimensionality dim = dataset.inferDescriptorDimensionality();
  ROS_ASSERT((size_t)dim.num_elements_.rows() == dataset.nameMapping("dmap").size());
  
  for(size_t i = 0; i < grids_.size(); ++i) {
    #pragma omp parallel for
    for(size_t j = 0; j < grids_[i].size(); ++j) {
      // applyNameTranslator initializes new descriptors with an empty vector of grids.
      // We have to fill these here.
      if(grids_[i][j].empty()) {
	grids_[i][j].resize(dim[j], (Grid*)NULL);
	for(size_t k = 0; k < grids_[i][j].size(); ++k) {
	  grids_[i][j][k] = new Grid;
	  grids_[i][j][k]->initialize(num_cells_[i], j, k, dataset);
	}
      }
    }
  }
}

void GridClassifier::setZero()
{
  prior_.setZero();
  
  for(size_t i = 0; i < grids_.size(); ++i)
    for(size_t j = 0; j < grids_[i].size(); ++j)
      for(size_t k = 0; k < grids_[i][j].size(); ++k)
        grids_[i][j][k]->cells_.setZero();
}

void GridClassifier::setZero(int c)
{
  prior_(c) = 0;

  for(size_t i = 0; i < grids_.size(); ++i)
    for(size_t j = 0; j < grids_[i].size(); ++j)
      for(size_t k = 0; k < grids_[i][j].size(); ++k)
        grids_[i][j][k]->cells_.row(c).setZero();
}

Label GridClassifier::classify(const Instance& instance) const
{
  Label prediction = prior_;
  for(size_t i = 0; i < instance.size(); ++i)
    if(instance[i])
      classify(*instance[i], i, &prediction);

  return prediction;
}

Label GridClassifier::classify(const std::vector<const Eigen::VectorXf*>& descriptors) const
{
  Label prediction = prior_;
  for(size_t i = 0; i < descriptors.size(); ++i)
    if(descriptors[i])
      classify(*descriptors[i], i, &prediction);
  
  return prediction;
}

void GridClassifier::classify(const Eigen::VectorXf& descriptor, size_t id, Label* prediction) const
{
  for(int i = 0; i < descriptor.rows(); ++i) { 
    float val = descriptor(i);
    for(size_t j = 0; j < grids_.size(); ++j) {
      ROS_ASSERT(grids_[j][id][i]);
      ROS_ASSERT((int)grids_[j][id].size() == descriptor.rows());
      const Grid& grid = *grids_[j][id][i];
      int idx = grid.getCellIdx(val);
      for(int k = 0; k < prediction->rows(); ++k)
	prediction->coeffRef(k) += grid.cells_.coeffRef(k, idx);
    }
  }
}

size_t GridClassifier::numCells() const
{
  size_t num = 0;
  for(size_t i = 0; i < grids_.size(); ++i) {
    for(size_t j = 0; j < grids_[i].size(); ++j) {
      for(size_t k = 0; k < grids_[i][j].size(); ++k) {
        ROS_ASSERT(grids_[i][j][k]);
        num += grids_[i][j][k]->cells_.cols();
      }
    }
  }
  return num;
}

size_t GridClassifier::numElements() const
{
  ROS_ASSERT(!grids_.empty());
  size_t num = 0;
  for(size_t j = 0; j < grids_[0].size(); ++j)
    num += grids_[0][j].size();
  return num;
}

// ci.didx_ must be set correctly.  The other members of ci can be ignored by the user of this function.
void GridClassifier::computeCellIndexWeighting(const Label& annotation,
                                               const Eigen::VectorXf& descriptor,
                                               CellIndex ci,
                                               std::vector<CellIndex>* index,
                                               std::vector<double>* ci_weights) const
{
  for(ci.eidx_ = 0; ci.eidx_ < descriptor.rows(); ++ci.eidx_) { 
    float val = descriptor(ci.eidx_);
    for(ci.ridx_ = 0; ci.ridx_ < (int)grids_.size(); ++ci.ridx_) {
      ROS_ASSERT(grids_[ci.ridx_][ci.didx_][ci.eidx_]);
      ROS_ASSERT((int)grids_[ci.ridx_][ci.didx_].size() == descriptor.rows());
      const Grid& grid = *grids_[ci.ridx_][ci.didx_][ci.eidx_];
      ci.cidx_ = grid.getCellIdx(val);

      //double weight = TBSSL_EXP(-annotation.dot(grid.cells_.col(ci.cidx_))) / descriptor.rows();
      double weight = 1.0 / descriptor.rows();
      ci_weights->push_back(weight);
      index->push_back(ci);
    }
  }
}

void GridClassifier::computeCellIndexWeighting(const Instance& instance,
                                               std::vector<CellIndex>* index,
                                               std::vector<double>* ci_weights) const
{
  index->clear();
  ci_weights->clear();

  CellIndex ci;
  for(ci.didx_ = 0; ci.didx_ < (int)instance.size(); ++ci.didx_)
    if(instance[ci.didx_])
      computeCellIndexWeighting(instance.label_, *instance[ci.didx_], ci, index, ci_weights);

  ROS_ASSERT(index->size() == ci_weights->size());
}

void GridClassifier::serialize(std::ostream& out) const
{
  serializeNameMappings(out);
  eigen_extensions::serialize(prior_, out);
  
  eigen_extensions::serializeScalar(num_cells_.size(), out);
  for(size_t i = 0; i < num_cells_.size(); ++i)
    eigen_extensions::serializeScalar(num_cells_[i], out);
  
  eigen_extensions::serializeScalar(grids_.size(), out);
  for(size_t i = 0; i < grids_.size(); ++i) {
    eigen_extensions::serializeScalar(grids_[i].size(), out);
    for(size_t j = 0; j < grids_[i].size(); ++j) {
      eigen_extensions::serializeScalar(grids_[i][j].size(), out);
      for(size_t k = 0; k < grids_[i][j].size(); ++k) {
	grids_[i][j][k]->serialize(out);
      }
    }
  }
  
}

void GridClassifier::deserialize(std::istream& in)
{
  deserializeNameMappings(in);
  eigen_extensions::deserialize(in, &prior_);

  size_t sz;
  eigen_extensions::deserializeScalar(in, &sz);
  num_cells_.resize(sz);
  for(size_t i = 0; i < num_cells_.size(); ++i)
    eigen_extensions::deserializeScalar(in, &num_cells_[i]);
  
  grids_.clear();
  size_t num;
  eigen_extensions::deserializeScalar(in, &num);
  grids_.resize(num);
  for(size_t i = 0; i < grids_.size(); ++i) {
    eigen_extensions::deserializeScalar(in, &num);
    grids_[i].resize(num);
    for(size_t j = 0; j < grids_[i].size(); ++j) {
      eigen_extensions::deserializeScalar(in, &num);
      grids_[i][j].resize(num);
      for(size_t k = 0; k < grids_[i][j].size(); ++k) {
	grids_[i][j][k] = new Grid;
	in >> *grids_[i][j][k];
      }
    }
  }
}

void GridClassifier::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  if(id == "cmap") {
    translator.translate(&prior_);
  }
  else if(id == "dmap")
    for(size_t i = 0; i < grids_.size(); ++i)
      translator.translate(&grids_[i], vector<Grid*>());
  else
    ROS_ASSERT(0);
}

std::string GridClassifier::status(const std::string& prefix, bool show_namemappings) const
{
  ostringstream oss;

  if(show_namemappings) {
    cout << "cmap: " << endl;
    cout << nameMapping("cmap").status("  ") << endl;
    cout << "dmap: " << endl;
    cout << nameMapping("dmap").status("  ") << endl;
  }

  oss << prefix << "Num cells: ";
  for(size_t i = 0; i < num_cells_.size(); ++i)
    oss << " " << num_cells_[i];
  oss << endl;

  oss << prefix << "Priors:" << endl;
  for(size_t i = 0; i < nameMapping("cmap").size(); ++i)
    oss << prefix << "  " << nameMapping("cmap").toName(i) << " " << prior_(i) << endl;

  return oss.str();
}

std::string GridClassifier::debug(const std::vector<const Eigen::VectorXf*>& descriptors) const
{
  ostringstream oss;
  oss << fixed << setprecision(3);
  
  for(size_t i = 0; i < nameMapping("cmap").size(); ++i)
    oss << setw(15) << nameMapping("cmap").toName(i);
  oss << setw(70) << "Descriptor space" << endl;
  oss << "----------------------------------------------------------------------------------------------------" << endl;
  for(int i = 0; i < prior_.rows(); ++i)
    oss << setw(15) << prior_(i);
  oss << setw(70) << "Prior" << endl;

  ROS_ASSERT(nameMapping("dmap").size() == descriptors.size());
  Label total = prior_;
  for(size_t i = 0; i < descriptors.size(); ++i) {
    Label prediction = VectorXf::Zero(prior_.rows());
    if(descriptors[i])
      classify(*descriptors[i], i, &prediction);
    total += prediction;
    for(int j = 0; j < prediction.rows(); ++j)
      oss << setw(15) << prediction(j);
    oss << setw(70) << nameMapping("dmap").toName(i) << endl;
  }
  for(int i = 0; i < total.rows(); ++i)
    oss << setw(15) << total(i);
  oss << setw(70) << "Total" << endl;

  return oss.str();
}

Eigen::ArrayXf GridClassifier::sparsity() const
{
  float total = 0;
  ArrayXf num_zeros = ArrayXf::Zero(nameMapping("cmap").size());
  for(size_t i = 0; i < grids_.size(); ++i) {
    for(size_t j = 0; j < grids_[i].size(); ++j) {
      for(size_t k = 0; k < grids_[i][j].size(); ++k) {
        const Grid* grid = grids_[i][j][k];
        ROS_ASSERT(grid);
        const MatrixXf& cells = grid->cells_;
        num_zeros += (cells.array() == 0).cast<float>().rowwise().sum();
        total += cells.cols();
      }
    }
  }
  return num_zeros / total;
}

/************************************************************
 * Grid
 ************************************************************/
  
Grid::Grid() :
  min_(FLT_MAX),
  max_(-FLT_MAX),
  width_(0),
  inv_width_(-1)
{
}

Grid::Grid(const Grid& other) :
  SharedLockable(),
  cells_(other.cells_),
  num_cells_(other.num_cells_),
  descriptor_id_(other.descriptor_id_),
  element_id_(other.element_id_),
  min_(other.min_),
  max_(other.max_),
  width_(other.width_),
  inv_width_(other.inv_width_)
{
}

Eigen::VectorXf Grid::bins() const
{
  VectorXf bins(cells_.cols());
  for(int i = 0; i < bins.rows(); ++i)
    bins(i) = min_ + i * width_;
  return bins;
}

bool Grid::operator==(const Grid& other)
{
  if(cells_ != other.cells_)
    return false;
  if(num_cells_ != other.num_cells_)
    return false;
  if(descriptor_id_ != other.descriptor_id_)
    return false;
  if(element_id_ != other.element_id_)
    return false;
  if(min_ != other.min_)
    return false;
  if(max_ != other.max_)
    return false;
  if(width_ != other.width_)
    return false;
  if(inv_width_ != other.inv_width_)
    return false;

  return true;
}

void Grid::initialize(size_t num_cells, size_t descriptor_id, size_t element_id, const TrackDataset& dataset)
{
  element_id_ = element_id;
  num_cells_ = num_cells;
  descriptor_id_ = descriptor_id;
	
  bool verbose = false;
  if(rand() % 1000 == 0)
    verbose = true;

  if(verbose)
    ROS_DEBUG_STREAM("Initializing Grid using " << dataset.totalInstances() << " instances.");

  vector<double> vals;
  vals.reserve(dataset.totalInstances());
  for(size_t i = 0; i < dataset.size(); ++i)
    for(size_t j = 0; j < dataset[i].size(); ++j)
      if(dataset[i][j][descriptor_id_])
        vals.push_back(dataset[i][j][descriptor_id_]->coeffRef(element_id_));

  sort(vals.begin(), vals.end());  // ascending
  // Assume 1% of the values on the edges are noise.
  double frac = 0.01;
  min_ = vals[vals.size() * frac];
  max_ = vals[vals.size() - 1 - vals.size() * frac];
  double range = max_ - min_;
  // Expand the range by that same percent on either side.
  min_ -= range * frac;
  max_ += range * frac;
    
  // -- Choose a bin width.
  width_ = (max_ - min_) / (double)num_cells;
  inv_width_ = 1.0 / width_;
  if(verbose)
    ROS_DEBUG_STREAM("Cell width set to " << width_);
    
  // -- Allocate cells.
  cells_ = MatrixXf::Zero(dataset.nameMapping("cmap").size(), num_cells);
}
  
void Grid::serialize(std::ostream& out) const
{
  eigen_extensions::serialize(cells_, out);
  eigen_extensions::serializeScalar(num_cells_, out);
  eigen_extensions::serializeScalar(descriptor_id_, out);
  eigen_extensions::serializeScalar(element_id_, out);
  eigen_extensions::serializeScalar(min_, out);
  eigen_extensions::serializeScalar(max_, out);
  eigen_extensions::serializeScalar(width_, out);
}
  
void Grid::deserialize(std::istream& in)
{
  eigen_extensions::deserialize(in, &cells_);
  eigen_extensions::deserializeScalar(in, &num_cells_);
  eigen_extensions::deserializeScalar(in, &descriptor_id_);
  eigen_extensions::deserializeScalar(in, &element_id_);
  eigen_extensions::deserializeScalar(in, &min_);
  eigen_extensions::deserializeScalar(in, &max_);
  eigen_extensions::deserializeScalar(in, &width_);
  inv_width_ = 1.0 / width_;
}
  
int Grid::getCellIdx(float val) const
{
  return std::min((int)cells_.cols() - 1, std::max(0, (int)((val - min_) * inv_width_)));
}

void Grid::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  if(id == "cmap") {
    translator.translateRows(&cells_);
    if(cells_.rows() == 0) {
      cells_.resize(translator.newSize(), num_cells_);
      cells_.setZero();
    }
  }
  else if(id == "dmap")
    translator.translate(&descriptor_id_);
  else
    ROS_ASSERT(0);
}


/************************************************************
 * StochasticLogisticTrainer
 ************************************************************/

typedef GridClassifier::StochasticLogisticTrainer GCSLT;

GCSLT::StochasticLogisticTrainer(GridClassifier::Ptr classifier) :
  gc_(classifier)
{
  ROS_ASSERT(gc_);
  ROS_ASSERT(gc_->prior_.rows() > 0);
  applyNameMappings(*gc_);
  classifier_ = gc_.get();
}

GCSLT::~StochasticLogisticTrainer()
{
  deallocateSchedulers();
}

void GCSLT::serialize(std::ostream& out) const
{
  eigen_extensions::serializeScalar(schedulers_.size(), out);
  for(size_t i = 0; i < schedulers_.size(); ++i) {
    ROS_ASSERT(schedulers_[i]);
    out << *schedulers_[i];
  }
}

void GCSLT::deallocateSchedulers()
{
  for(size_t i = 0; i < schedulers_.size(); ++i)
    if(schedulers_[i])
      delete schedulers_[i];
  schedulers_.clear();
}

void GCSLT::deserialize(std::istream& in)
{
  deallocateSchedulers();
  
  size_t num_sched;
  eigen_extensions::deserializeScalar(in, &num_sched);
  schedulers_.resize(num_sched);
  for(size_t i = 0; i < schedulers_.size(); ++i) {
    schedulers_[i] = new SqrtScheduler(ETA);
    in >> *schedulers_[i];
  }
}

void GCSLT::train(const Instance& instance)
{
  ROS_ASSERT(instance.label_.rows() == (int)gc_->nameMapping("cmap").size());
  ROS_ASSERT(!gc_->grids_.empty());  // Prevents training an uninitialized classifier.

  Label prediction = gc_->classify(instance);
  for(int i = 0; i < instance.label_.rows(); ++i)
    if(instance.label_(i) != 0)
      step(instance, prediction, i);
}

void GCSLT::train(const Instance& instance, const Label& prediction)
{
  // -- If the prediction does not match the annotation,
  //    and the instance annotation is non-zero,
  //    step that class problem.
  for(int i = 0; i < instance.label_.rows(); ++i)
    if(instance.label_.coeffRef(i) != 0)
      step(instance, prediction, i);
}

void GCSLT::step(const Instance& instance, const Label& prediction, int cid)
{
  // Dividing by the abs val makes this treat instances as having training example weights equal to the magnitude of instance.label_(cid).
  double margin = (instance.label_(cid) / fabs(instance.label_(cid))) * prediction(cid);
  if(margin > 13.0) {
    //ROS_WARN("*****");
    return;
  }
  
  double weight = 1.0 / (1.0 + TBSSL_EXP(margin));
  
  lockRead();
  double update = schedulers_[cid]->getLearningRate() * instance.label_(cid) * weight;
  unlockRead();

  // gc_->lockWrite();
  // gc_->prior_(cid) += update;
  // gc_->unlockWrite();

  for(size_t i = 0; i < instance.size(); ++i) {
    if(instance[i]) {
      for(int j = 0; j < instance[i]->rows(); ++j) {
	for(size_t k = 0; k < gc_->grids_.size(); ++k) {
	  Grid& grid = *gc_->grids_[k][i][j];
	  grid.lockWrite();
	  int idx = grid.getCellIdx(instance[i]->coeffRef(j));
	  grid.cells_(cid, idx) += update;
	  grid.unlockWrite();
	}
      }
    }
  }

  lockWrite();
  schedulers_[cid]->incrementInstances();
  unlockWrite();
}

void GCSLT::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  ROS_ASSERT(id == "cmap" || id == "dmap");
  // TODO: Removing a class results in a tiny memory leak.
  if(id == "cmap") {
    translator.translate(&schedulers_, (SqrtScheduler*)NULL);
    for(size_t i = 0; i < schedulers_.size(); ++i)
      if(!schedulers_[i])
	schedulers_[i] = new SqrtScheduler(ETA);
  }
}

std::string GCSLT::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "Num instances seen: " << endl;
  for(size_t i = 0; i < schedulers_.size(); ++i)
    oss << prefix << "  " << nameMapping("cmap").toName(i) << ": " << schedulers_[i]->getNumInstances() << endl;
  oss << prefix << "Learning rates: " << endl;
  for(size_t i = 0; i < schedulers_.size(); ++i)
    oss << prefix << "  " << nameMapping("cmap").toName(i) << ": " << schedulers_[i]->getLearningRate() << endl;
  return oss.str();
}

bool GCSLT::operator==(const GridClassifier::StochasticLogisticTrainer& other)
{
  if(schedulers_.size() != other.schedulers_.size())
    return false;
  for(size_t i = 0; i < schedulers_.size(); ++i) {
    if(schedulers_[i]->getLearningRate() != other.schedulers_[i]->getLearningRate())
      return false;
    if(schedulers_[i]->getNumInstances() != other.schedulers_[i]->getNumInstances())
      return false;
  }
  return true;
}

void GCSLT::resetSchedulers()
{
  for(size_t i = 0; i < schedulers_.size(); ++i)
    schedulers_[i]->reset();
}

void GCSLT::train(const std::vector<TrackDataset::ConstPtr>& datasets,
		  const std::vector<Indices>& indices)
{
  int batch_size = 100;
  ROS_ASSERT(classifier_);
  ROS_ASSERT(indices.size() == datasets.size());
  for(size_t i = 0; i < datasets.size(); ++i) {
    ROS_ASSERT(datasets[i]);
    if(i > 0)
      ROS_ASSERT(datasets[i]->nameMappingsAreEqual(*datasets[i-1]));
  }
  
  // TODO: Check that classifier has the same name mappings as the trainer.
  
  // -- Get total number of instances.
  size_t total_num_instances = 0;
  for(size_t i = 0; i < indices.size(); ++i) {
    ROS_ASSERT(nameMappingsAreEqual(*datasets[i]));
    for(size_t j = 0; j < indices[i].size(); ++j) {
      int trid = indices[i][j];
      if(datasets[i]->tracks_[trid]->size() > 0 && datasets[i]->label(trid).id() != -2)
	total_num_instances += datasets[i]->tracks_[trid]->size();
    }
  }
  cout << "Training on " << total_num_instances << " instances in " << datasets.size() << " datasets." << endl;

  // -- Build the index.
  vector<Index> index(total_num_instances);
  size_t idx = 0;
  for(size_t i = 0; i < indices.size(); ++i) {
    for(size_t j = 0; j < indices[i].size(); ++j) {
      int trid = indices[i][j];
      
      if(datasets[i]->tracks_[trid]->size() > 0 && datasets[i]->label(trid).id() == -2)
	continue;
      ROS_ASSERT(trid >= 0 && trid < (int)datasets[i]->tracks_.size());
      for(size_t k = 0; k < datasets[i]->tracks_[trid]->size(); ++k, ++idx)
	index[idx] = Index(i, trid, k);
    }
  }
  ROS_ASSERT(idx == total_num_instances);
  random_shuffle(index.begin(), index.end());

  // -- Mini-batch loop.
  vector<Label> predictions(index.size());
  idx = 0;

  size_t minibatch_threshold = 10000;
  if(index.size() < minibatch_threshold) {
    ROS_WARN_STREAM("Training on fewer than " << minibatch_threshold << " examples.  Using serial rather than mini-batch.");
    batch_size = 1;
  }
  
  while(idx < index.size()) {
    HighResTimer hrt;
    size_t end_idx = min(idx + batch_size, index.size());
    //cout << "Classifying " << idx << " to " << end_idx << endl;
    
    // -- Classify this batch in parallel.
    #pragma omp parallel for
    for(size_t i = idx; i < end_idx; ++i) {
      ROS_ASSERT(index[i].dataset_ >= 0 && index[i].dataset_ < (int)datasets.size());
      const Instance& instance = (*datasets[index[i].dataset_])[index[i].track_][index[i].frame_];
      predictions[i] = classifier_->classify(instance);
    }
  
    // -- Update the classifier in parallel.
    #pragma omp parallel for
    for(size_t i = idx; i < end_idx; ++i) {
      const Instance& instance = (*datasets[index[i].dataset_])[index[i].track_][index[i].frame_];
      train(instance, predictions[i]);
    }

    //cout << "Done classifying " << idx << " to " << end_idx << endl;
    idx += batch_size;
  }
}


/************************************************************
 * BoostingTrainer
 ************************************************************/

typedef GridClassifier::BoostingTrainer GCBT;

GCBT::BoostingTrainer(GridClassifier::Ptr classifier) :
  obj_thresh_(10),
  convergence_check_period_(10),
  max_iter_(0),
  verbose_(false),
  gamma_(0),
  gc_(classifier)
{
  ROS_ASSERT(gc_);
  ROS_ASSERT(gc_->prior_.rows() > 0);
  applyNameMappings(*gc_);
  classifier_ = gc_.get();
}

GCBT::~BoostingTrainer()
{
}

#define MAX_LOG_WEIGHT 100.0
inline void updateWeightsCol(size_t i, ArrayXXd& log_weights, ArrayXXd& weights)
{
  for(int j = 0; j < log_weights.rows(); ++j) {
    ROS_ASSERT(!isinf(log_weights(j, i)) && !isnan(log_weights(j, i)));
    log_weights(j, i) = min(log_weights(j, i), MAX_LOG_WEIGHT);
    weights(j, i) = TBSSL_EXP(log_weights(j, i));
  }
  // for(int j = 0; j < log_weights.rows(); ++j) {
  //   weights(j, i) = TBSSL_EXP(log_weights(j, i));
  // }
}


void GCBT::evaluateWC(int num_tr_ex, const std::vector<TrackDataset::ConstPtr>& datasets,
                      vector<size_t> ds, vector<size_t> ts, vector<size_t> fs,
                      const Eigen::ArrayXXd& weights, const Eigen::ArrayXXd& labels,
                      const CellIndex& c,
                      double* improvement, Eigen::ArrayXd* update, std::vector<bool>* inliers) const
{
  // -- Get inliers
  Grid& grid = *gc_->grids_[c.ridx_][c.didx_][c.eidx_];
  inliers->assign(num_tr_ex, false);
  for(int i = 0; i < num_tr_ex; ++i) {
    const Instance& inst = (*datasets[ds[i]])[ts[i]][fs[i]];
    if(!inst[c.didx_])
      continue;
    if(grid.getCellIdx((*inst[c.didx_])(c.eidx_)) == c.cidx_) {
      inliers->at(i) = true;
    }
  }
  
  // -- Compute update.
  *update = computeUpdate(*inliers, weights, labels);
  
  // -- Compute the improvement that would result.
  double imp = 0;
  // sum_i w_i (1 - exp(-y_i h(x_i))), vectorized.
  ArrayXd ones = ArrayXd::Ones(update->rows());
  for(size_t i = 0; i < inliers->size(); ++i)
    if(inliers->at(i))
      imp += (weights.col(i) * (ones - (-labels.col(i) * (*update)).exp())).sum();

  if(imp < 0)
    ROS_WARN_STREAM("GCBT::evaluateWC: improvement is " << imp);
  *improvement = imp;
}

void GCBT::train(const std::vector<TrackDataset::ConstPtr>& datasets,
		 const std::vector<Indices>& indices)
{
  for(size_t i = 0; i < datasets.size(); ++i) {
    ROS_ASSERT(datasets[i]);
    ROS_ASSERT(nameMappingsAreEqual(*datasets[i]));
    if(i > 0)
      ROS_ASSERT(datasets[i]->nameMappingsAreEqual(*datasets[i-1]));
    for(size_t j = 0; j < datasets[i]->size(); ++j)
      ROS_ASSERT((*datasets[i])[j].nameMappingsAreEqual(*datasets[i]));
  }
  
  // -- Build the index.
  size_t num_tr_ex = 0;
  for(size_t i = 0; i < indices.size(); ++i) {
    for(size_t j = 0; j < indices[i].size(); ++j) { 
      size_t t = indices[i][j];
      if((datasets[i]->label(t).array() != 0).any())
        num_tr_ex += (*datasets[i])[t].size();
    }
  }
  if(num_tr_ex == 0)
    return;
     
  vector<size_t> ds;  // datasets
  vector<size_t> ts;  // tracks
  vector<size_t> fs;  // frames
  ds.reserve(num_tr_ex);
  ts.reserve(num_tr_ex);
  fs.reserve(num_tr_ex);
  for(size_t i = 0; i < indices.size(); ++i) {
    for(size_t j = 0; j < indices[i].size(); ++j) {
      size_t t = indices[i][j];
      if((datasets[i]->label(t).array() != 0).any()) {
        for(size_t k = 0; k < (*datasets[i])[t].size(); ++k) {
          ds.push_back(i);
          ts.push_back(t);
          fs.push_back(k);
        }
      }
    }
  }    
  
  // -- Compute the boosting weights distribution over the examples for each class.
  ArrayXXd log_weights = ArrayXXd::Zero(nameMapping("cmap").size(), num_tr_ex);
  ArrayXXd weights(nameMapping("cmap").size(), num_tr_ex);
  ArrayXXd labels(nameMapping("cmap").size(), num_tr_ex);
  #pragma omp parallel for
  for(size_t i = 0; i < num_tr_ex; ++i) {
    const Instance& inst = (*datasets[ds[i]])[ts[i]][fs[i]];
    labels.col(i) = inst.label_.sign().array().cast<double>();  // -1, 0, +1

    // 2013-11-26: Got some suspicious output from below indicating that unlabeled
    // examples in datasets[...] were being used as negative training examples.
    // We're not using weighting in the training sets anymore, so the instance
    // labels should be exactly -1, 0, and +1.  Otherwise we could get bad behavior
    // where, e.g., a label of -1e-13 gets set to -1 by the above.
    for(int j = 0; j < inst.label_.rows(); ++j) {
      float val = inst.label_.coeffRef(j);
      if(!(val == -1 || val == 0 || val == +1)) {
        cout << "label value is " << val << " but should be either 0 or +/-1." << endl;
        ROS_ASSERT(0);
      }
    }

    // Log weights includes the importance weight embedded in the label.
    log_weights.col(i) = -labels.col(i) * gc_->classify(inst).array().cast<double>();
    for(int j = 0; j < inst.label_.rows(); ++j)
      if(inst.label_(j) != 0)
        log_weights(j, i) += log(fabs(inst.label_(j)));

    updateWeightsCol(i, log_weights, weights);
  }

  // -- Print out class proportions.
  cout << "GCBT::train" << endl;
  // cout << "Positive examples: " << (labels == 1).rowwise().sum() << endl;
  // cout << "Negative examples: " << (labels == -1).rowwise().sum() << endl;
  ArrayXi pos = ArrayXi::Zero(labels.rows());
  ArrayXi neg = ArrayXi::Zero(labels.rows());
  for(int i = 0; i < labels.cols(); ++i) {
    for(int j = 0; j < labels.rows(); ++j) {
      if(labels(j, i) == 1)
        ++pos(j);
      else if(labels(j, i) == -1)
        ++neg(j);
    }
  }
  cout << "Positive examples: " << pos.transpose() << endl;
  cout << "Negative examples: " << neg.transpose() << endl;
  
  // -- Main loop adding new weak classifiers.
  std::vector<double> objs;
  objs.reserve(10000);
  //objs.push_back(weights.sum() / ((double)weights.rows() * weights.cols()));
  objs.push_back(weights.sum());
  if(verbose_)
    cout << "Initial objective: " << objs.back() << endl;
  int iter = 0;
  eigen_extensions::UniformSampler sampler(0);

  int chunk_size = 24;
  //int chunk_size = sysconf(_SC_NPROCESSORS_ONLN);
  vector<CellIndex> cell_indices(chunk_size);
  vector<bool> valid(chunk_size);
  vector<double> improvements(chunk_size);
  vector<ArrayXd> updates(chunk_size); //cell_indices.size(), ArrayXd::Zero(nameMapping("cmap").size()));
  vector< vector<bool> > inliers(chunk_size);
  
  while(true) {
    HighResTimer hrt_total("Total");
    hrt_total.start();
    
    // -- Choose random weak classifiers.
    HighResTimer hrt_sampling("Sampling weak classifiers"); hrt_sampling.start();
    VectorXd weights_for_sampling = weights.colwise().sum().matrix();
    valid.assign(chunk_size, false);
    for(int i = 0; i < chunk_size; ++i) {
      // Sample from the training examples weights distribution.
      int tridx = eigen_extensions::weightedSample(weights_for_sampling);
      // cout << "Sampled training example with weight " << weights_for_sampling(tridx)
      //      << ".  Mean: " << weights_for_sampling.sum() / weights_for_sampling.rows() << endl;
      const Instance& instance = (*datasets[ds[tridx]])[ts[tridx]][fs[tridx]];

      // Sample a random grid cell that affects that training example.
      CellIndex& c = cell_indices[i];
      c.ridx_ = sampler.sample() % gc_->grids_.size();
      c.didx_ = sampler.sample() % gc_->grids_[c.ridx_].size();
      c.eidx_ = sampler.sample() % gc_->grids_[c.ridx_][c.didx_].size();
      ROS_ASSERT(gc_->grids_[c.ridx_][c.didx_][c.eidx_]);
      // If this descriptor is missing, move on.
      if(!instance[c.didx_])
        continue;
      ROS_ASSERT(c.eidx_ < instance[c.didx_]->rows() && c.eidx_ >= 0);
      Grid& grid = *gc_->grids_[c.ridx_][c.didx_][c.eidx_];
      c.cidx_ = grid.getCellIdx(instance[c.didx_]->coeffRef(c.eidx_));
      valid[i] = true;
    }
    hrt_sampling.stop();

    // -- Evaluate them all in parallel.
    HighResTimer hrt_boosting("Evaluating wcs"); hrt_boosting.start();
    improvements.assign(cell_indices.size(), 0.0);
    #pragma omp parallel for
    for(size_t i = 0; i < cell_indices.size(); ++i) {
      if(valid[i]) {
        evaluateWC(num_tr_ex, datasets, ds, ts, fs,
                   weights, labels,
                   cell_indices[i], &improvements[i], &updates[i], &inliers[i]);
      }
    }
    hrt_boosting.stop();
    
    // -- Apply the best one.
    size_t best = distance(improvements.begin(), max_element(improvements.begin(), improvements.end()));
    for(size_t i = 0; i < improvements.size(); ++i)
      ROS_ASSERT(improvements[i] <= improvements[best]);

    HighResTimer hrt_weights("Updating weights");
    if(improvements[best] > 0) {
      const CellIndex& c = cell_indices[best];
      ROS_ASSERT(gc_->grids_[c.ridx_][c.didx_][c.eidx_]);
      Grid& grid = *gc_->grids_[c.ridx_][c.didx_][c.eidx_];
      grid.cells_.col(c.cidx_) += updates[best].matrix().cast<float>();

      // -- Update the log weights.
      hrt_weights.start();
      #pragma omp parallel for
      for(size_t i = 0; i < inliers[best].size(); ++i) {
        if(inliers[best][i]) {
          log_weights.col(i) += -labels.col(i) * updates[best];
          updateWeightsCol(i, log_weights, weights);
        }
      }
      hrt_weights.stop();
    }

    // -- Check that the objective didn't go up.
    //double obj = weights.sum() / ((double)weights.rows() * weights.cols());
    double obj = weights.sum();
    ROS_ASSERT(!isnan(obj));
    hrt_total.stop();
    
    if(obj > objs.back())
      ROS_WARN_STREAM("Objective did not decrease.  Old minus new: " << objs.back() - obj);

    if(verbose_ && (iter % 100) == 0) { 
      cout << "------------------------------------------------------------" << endl;
      cout << "Iteration " << iter << endl;
      cout << "Objective: " << obj << endl;
      cout << "Improvement: " << objs.back() - obj << endl;
      if(objs.size() > convergence_check_period_)
        cout << "Improvement over last " << convergence_check_period_ << " wcs: " << *(objs.end() - convergence_check_period_) - obj << endl;
      cout << "Improvement over last " << convergence_check_period_ << " iterations must be below this amount to stop: " << obj_thresh_ << endl;      
      cout << "Chunk size: " << chunk_size << endl;
      cout << "Num evaluated: " << improvements.size() << endl;
      cout << hrt_boosting.reportMilliseconds() << endl;
      cout << hrt_weights.reportMilliseconds() << endl;
      cout << hrt_sampling.reportMilliseconds() << endl;
      cout << hrt_total.reportMilliseconds() << endl;
    }
    
    // -- Stop if objective hasn't gone down enough in a while.
    if(objs.size() > convergence_check_period_ && *(objs.end() - convergence_check_period_) - obj < obj_thresh_)
      break;
        
    if(max_iter_ > 0 && iter > max_iter_)
      break;
    ++iter;

    objs.push_back(obj);
  }
}

ArrayXd GCBT::computeUpdate(const vector<bool>& inliers, const ArrayXXd& weights, const ArrayXXd& labels) const
{
  // -- Compute the new weak classifier for each class.
  //    cell += sum of weights in this cell * label / sum of weights in this cell.
  ArrayXd numerator = ArrayXd::Zero(nameMapping("cmap").size());
  ArrayXd denominator = ArrayXd::Zero(nameMapping("cmap").size());
  for(size_t i = 0; i < inliers.size(); ++i) {
    if(inliers[i]) { 
      numerator += weights.col(i) * labels.col(i);
      denominator += weights.col(i) * labels.col(i).abs();  // Respect labels with 0 entries.
    }
  }
  ArrayXd update = numerator / denominator;
  
  // -- Sometimes the denominator is zero.
  for(int i = 0; i < update.rows(); ++i)
    if(isnan(update.coeffRef(i)))
      update.coeffRef(i) = 0;
  
  // -- Apply soft thresholding regularization.
  if(gamma_ != 0) {
    for(int i = 0; i < update.rows(); ++i) {
      if(update.coeffRef(i) > gamma_)
        update.coeffRef(i) -= gamma_;
      else if(update.coeffRef(i) < -gamma_)
        update.coeffRef(i) += gamma_;
      else
        update.coeffRef(i) = 0;
    }
  }

  return update;
}

void GCBT::serialize(std::ostream& out) const
{
  eigen_extensions::serializeScalar(obj_thresh_, out);
  eigen_extensions::serializeScalar(convergence_check_period_, out);
  eigen_extensions::serializeScalar(verbose_, out);
  eigen_extensions::serializeScalar(gamma_, out);
}

void GCBT::deserialize(std::istream& in)
{
  eigen_extensions::deserializeScalar(in, &obj_thresh_);
  eigen_extensions::deserializeScalar(in, &convergence_check_period_);
  eigen_extensions::deserializeScalar(in, &verbose_);
  eigen_extensions::deserializeScalar(in, &gamma_);
}

std::string GCBT::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "Boosting trainer" << endl;
  oss << prefix << "Objective threshold: " << obj_thresh_ << endl;
  oss << prefix << "Convergence check period: " << convergence_check_period_ << endl;
  oss << prefix << "Verbose: " << verbose_ << endl;
  oss << prefix << "Regularization (gamma_): " << gamma_ << endl;
  return oss.str();
}

bool GCBT::operator==(const GridClassifier::BoostingTrainer& other)
{
  if(obj_thresh_ != other.obj_thresh_)
    return false;
  if(convergence_check_period_ != other.convergence_check_period_)
    return false;
  if(verbose_ != other.verbose_)
    return false;
  if(gamma_ != other.gamma_)
    return false;

  return true;
}

void GCBT::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  
}

void GCBT::train(const Instance& instance)
{
  ROS_ASSERT(0);
}

void GCBT::train(const Instance& instance, const Label& prediction)
{
  ROS_ASSERT(0);
}


/************************************************************
 * BisectionBoostingTrainer
 ************************************************************/

// typedef GridClassifier::BisectionBoostingTrainer GCBBT;

// GCBBT::BisectionBoostingTrainer(GridClassifier::Ptr classifier) :
//   GCBT(classifier)
// {
// }

// std::string GCBBT::status(const std::string& prefix) const
// {
//   ostringstream oss;
//   oss << prefix << "Bisection boosting trainer" << endl;
//   oss << prefix << "Objective threshold: " << obj_thresh_ << endl;
//   oss << prefix << "Number of times sub-threshold to terminate: " << noimp_thresh_ << endl;
//   oss << prefix << "Verbose: " << verbose_ << endl;
//   oss << prefix << "Regularization (gamma_): " << gamma_ << endl;
//   return oss.str();
// }

// class WeakClassifierObjective : public ScalarFunction
// {
// public:
//   double gamma_;
//   const Eigen::ArrayXd* weights_;
//   const Eigen::ArrayXd* labels_;

//   WeakClassifierObjective(double gamma, const Eigen::ArrayXd* weights, const Eigen::ArrayXd* labels) :
//     gamma_(gamma),
//     weights_(weights),
//     labels_(labels)
//   {
//   }
     
//   double eval(const VectorXd& x) const
//   {
//     ROS_ASSERT(x.rows() == 1);
//     ROS_ASSERT(weights_->rows() == labels_->rows());

//     double alpha = x(0);
//     double obj = 0;
//     //obj += gamma_ * fabs(alpha);
//     obj += gamma_ * alpha * alpha;
//     for(int i = 0; i < weights_->rows(); ++i)
//       obj += weights_->coeffRef(i) * TBSSL_EXP(-labels_->coeffRef(i) * alpha);
//     return obj;
//   }
// };

// // Technically the gradient does not exist for alpha = 0,
// // so this is actually returning a subgradient, but for a 1D bisection problem
// // this should be fine.
// class WeakClassifierGradient : public VectorFunction
// {
// public:
//   double gamma_;
//   const Eigen::ArrayXd* weights_;
//   const Eigen::ArrayXd* labels_;

//   WeakClassifierGradient(double gamma, const Eigen::ArrayXd* weights, const Eigen::ArrayXd* labels) :
//     gamma_(gamma),
//     weights_(weights),
//     labels_(labels)
//   {
//   }
  
//   VectorXd eval(const VectorXd& x) const
//   {
//     ROS_ASSERT(x.rows() == 1);
//     ROS_ASSERT(weights_->rows() == labels_->rows());

//     double alpha = x(0);
//     double subgradient = 0;
//     //subgradient += gamma_ * sign(alpha);
//     subgradient += 2 * gamma_ * alpha;
    
//     for(int i = 0; i < weights_->rows(); ++i)
//       subgradient -= labels_->coeffRef(i) * weights_->coeffRef(i) * TBSSL_EXP(-labels_->coeffRef(i) * alpha);

//     VectorXd result(1);
//     result(0) = subgradient;
//     return result;
//   }
// };


// ArrayXd GCBBT::computeUpdate(const vector<bool>& inside, const ArrayXXd& weights, const ArrayXXd& labels) const
// {
//   ROS_ASSERT(0);
//   return ArrayXd();
  
//   // ROS_ASSERT(inside_weights.rows() == inside_labels.rows());
//   // ROS_ASSERT(inside_weights.cols() == inside_labels.cols());

//   // ArrayXd update(inside_weights.rows());
//   // for(int i = 0; i < inside_weights.rows(); ++i) { 
//   //   ArrayXd weights_class = inside_weights.row(i);
//   //   ArrayXd labels_class = inside_labels.row(i);
//   //   WeakClassifierObjective objective(gamma_, &weights_class, &labels_class);
//   //   WeakClassifierGradient gradient(gamma_, &weights_class, &labels_class);

//   //   double tol = 0.001;
//   //   double min = -10;
//   //   double max = 10;
//   //   int max_iters = 100;
//   //   BisectionSolver solver(&objective, &gradient, tol, min, max, max_iters);
//   //   //solver.debug_ = true;
//   //   update(i) = solver.solve();
//   // }
  
//   //return update;
// }
