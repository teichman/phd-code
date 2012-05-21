#include <multibooster/l1tree.h>
#include <multibooster/multibooster.h>

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN;
  

L1Tree::L1Tree(const std::vector<WeakClassifier*>& pwcs, const L1TreeParams& params, MultiBoosterDataset* mbd, size_t dspace) :
  parent_(NULL),
  pwcs_(pwcs),
  rchild_(NULL),
  lchild_(NULL),
  params_(params),
  depth_(0),
  mbd_(mbd),
  dspace_(dspace)
{
  growTree();
}

L1Tree::~L1Tree()
{
  if(rchild_)
    delete rchild_;
  if(lchild_)
    delete lchild_;
}

void L1Tree::query(const Eigen::VectorXf& point, std::vector<WeakClassifier*>* activated, int* num_evaluated) const
{  
  if(!isLeaf()) {
    if(projector_.dot(point) < threshold_)
      lchild_->query(point, activated, num_evaluated);
    else
      rchild_->query(point, activated, num_evaluated);
  }
  else {
    for(size_t i = 0; i < pwcs_.size(); ++i) {
      ++(*num_evaluated);
      float sd = distL1(point, pwcs_[i]->center_);
      if(sd <= pwcs_[i]->theta_)
	activated->push_back(pwcs_[i]);
    }
  }
}

bool L1Tree::isLeaf() const
{
  if(!rchild_ && !lchild_)
    return true;
  else
    return false;
}

L1Tree::L1Tree(L1Tree* parent, const std::vector<WeakClassifier*>& pwcs) :
  parent_(parent),
  pwcs_(pwcs),
  rchild_(NULL),
  lchild_(NULL),
  params_(parent->params_),
  depth_(parent->depth_ + 1),
  mbd_(parent->mbd_),
  dspace_(parent->dspace_)
{
  cout << "Adding a new node with " << pwcs.size() << " WCs in it." << endl;

  growTree();
}

void L1Tree::growTree()
{
  if(parent_ && pwcs_.size() == parent_->pwcs_.size()) { 
    doneGrowing();
    return;
  }
  
  if(pwcs_.size() < params_.min_wcs_) { 
    doneGrowing();
    return;
  }

  if(depth_ == params_.max_depth_) { 
    doneGrowing();
    return;
  }

  // -- Compute the projection vector.
  int dim = pwcs_[0]->center_.rows();
  MatrixXd centers(dim, pwcs_.size());
  for(size_t i = 0; i < pwcs_.size(); ++i)
    centers.col(i) = pwcs_[i]->center_.cast<double>();

  VectorXd pc = bruteForceProjectionSearch(centers);
  projector_ = pc.cast<float>();
  //VectorXd pc = firstPrincipalComponent(centers);
//   projector_ = createProjectionDirection(pc).cast<float>();
  
//   // -- If this node has the same weak classifiers as the parent,
//   //    then choose a projection vector that is different from all ancestors with the same
//   //    set of WCs.
//   if(parent_ && (pwcs_.size() == parent_->pwcs_.size())) {
//     vector<VectorXf> parent_projectors;
//     getProjectorsOfParentsWithSameWCs(&parent_projectors);
//     assert(!parent_projectors.empty());
    
//     if(isDuplicateProjector(projector_, parent_projectors)) { 
//       MatrixXd pcs = pca(centers);
//       bool valid = false;
//       for(int i = 0; i < pcs.cols(); ++i) {
// 	projector_ = createProjectionDirection(pcs.col(i)).cast<float>();
// 	if(!isDuplicateProjector(projector_, parent_projectors)) {
// 	  valid = true;
// 	  break;
// 	}
//       }

//       if(!valid) {
// 	doneGrowing();
// 	return;
//       }
//     }
//   }

    
  // -- Set the threshold based on the expected num WCs eliminated.
  vector<WeakClassifier*> left;
  vector<WeakClassifier*> right;
  double expected_num_eliminated = setThreshold(&left, &right);
  if(expected_num_eliminated < params_.min_expected_num_eliminated_) {
    doneGrowing();
    return;
  }

  // -- Recurse.
  cout << "Expected num eliminated: " << expected_num_eliminated << endl;
  cout << "Adding a new split with " << left.size() << " on the left and " << right.size() << " on the right." << endl;
  lchild_ = new L1Tree(this, left);
  rchild_ = new L1Tree(this, right);
}

void L1Tree::doneGrowing() const
{
  cout << "Found a leaf at depth " << depth_ << endl;
}

void L1Tree::getProjectorsOfParentsWithSameWCs(std::vector<Eigen::VectorXf>* projectors) const
{
  if(parent_ && (pwcs_.size() == parent_->pwcs_.size())) {
    projectors->push_back(parent_->projector_);
    parent_->getProjectorsOfParentsWithSameWCs(projectors);
  }
}

double L1Tree::setThreshold(vector<WeakClassifier*>* left, vector<WeakClassifier*>* right)
{
  vector<double> boundaries;
  boundaries.reserve(pwcs_.size() * 3);
  for(size_t i = 0; i < pwcs_.size(); ++i) {
    boundaries.push_back(projector_.dot(pwcs_[i]->center_) + pwcs_[i]->projection_threshold_);
    boundaries.push_back(projector_.dot(pwcs_[i]->center_) - pwcs_[i]->projection_threshold_);
    boundaries.push_back(projector_.dot(pwcs_[i]->center_));
  }
  sort(boundaries.begin(), boundaries.end()); // Ascending order.

  // -- Precompute the projections of the tuning set.
  VectorXd data_projections(mbd_->objs_.size());
  for(size_t i = 0; i < mbd_->objs_.size(); ++i)
    data_projections(i) = mbd_->objs_[i]->descriptors_[dspace_].vector->dot(projector_);
    
  // -- Find the best separator.
  double best_expected_num_elim = 0;
  for(size_t i = 1; i < boundaries.size(); ++i) {
    double sep = (boundaries[i-1] + boundaries[i]) / 2.0;
    double num_entirely_left = 0;
    double num_entirely_right = 0;
    for(size_t j = 0; j < pwcs_.size(); ++j) {
      double proj = projector_.dot(pwcs_[j]->center_);

      if(proj + pwcs_[j]->projection_threshold_ < sep)
	++num_entirely_left;
      else if(proj - pwcs_[j]->projection_threshold_ > sep) 
	++num_entirely_right;
    }

    double num_data_left = (data_projections.cwise() < sep).count();
    double num_data_right = (data_projections.cwise() >= sep).count();
    assert((int)(num_data_left + num_data_right) == data_projections.rows());

    double expected_num_elim = num_data_left / (double)(mbd_->objs_.size()) * num_entirely_right + num_data_right / (double)(mbd_->objs_.size()) * num_entirely_left;
	      
    if(expected_num_elim > best_expected_num_elim) { 
      best_expected_num_elim = expected_num_elim;
      threshold_ = sep;
    }
  }

  // -- Determine which WCs are on which side.
  for(size_t i =  0; i < pwcs_.size(); ++i) {
    double proj = projector_.dot(pwcs_[i]->center_);
    if(proj + pwcs_[i]->projection_threshold_ < threshold_) { 
      left->push_back(pwcs_[i]);
    }
    else if(proj - pwcs_[i]->projection_threshold_ > threshold_) { 
      right->push_back(pwcs_[i]);
    }
    else {
      left->push_back(pwcs_[i]);
      right->push_back(pwcs_[i]);
    }
  }

  return best_expected_num_elim;
}


MatrixXd pca(const MatrixXd& instances, int num)
{
  if(num > instances.rows() || num == -1)
    num = instances.rows();

  VectorXd mean = instances.rowwise().sum() / (double)instances.cols();
  MatrixXd centered(instances.rows(), instances.cols());
  for(int i = 0; i < centered.cols(); ++i)
    centered.col(i) = instances.col(i) - mean;
	
  MatrixXd xxt = centered * centered.transpose();
    
  Eigen::SVD<MatrixXd> svd(xxt); // xxt is PSD.
  VectorXd evals = svd.singularValues();
  int idx = -1;
  evals.maxCoeff(&idx);
  assert(idx == 0);
  for(int i = 1; i < evals.rows(); ++i) {
    assert(evals(i - 1) >= evals(i));
  }

  MatrixXd all = svd.matrixU();
  MatrixXd principal(instances.rows(), num);
  for(int i = 0; i < principal.cols(); ++i)
    principal.col(i) = all.col(i);
  
  return principal;
}


VectorXd bruteForceProjectionSearch(const MatrixXd& instances)
{
  
  int dim = instances.rows();
  VectorXd projector = VectorXd::Ones(dim);
  VectorXd best_projector;
  double best_var = -FLT_MAX;
  for(int i = 0; i < pow(2, dim - 1); ++i) {
    // -- Create the projector.
    for(int j = 0; j < dim; ++j) {
      if((int)floor((double)i / pow(2.0, (double)j)) % 2 == 1)
	projector(j) = -1.0 / sqrt((double)dim);
      else
	projector(j) = 1 / sqrt((double)dim);
    }

    // -- Compute the variance.
    double var = 0;
    VectorXd projections = projector.transpose() * instances;
    assert(projections.cols() == 1);
    double mean = projections.sum() / (double)(projections.rows());
    for(int j = 0; j < projections.rows(); ++j)
      var += pow(projections.coeff(j) - mean, 2.0);
    var /= (double)(projections.rows());
      
    // -- Save it if it's the best so far.
    if(var > best_var) { 
      best_var = var;
      best_projector = projector;
    }
  }
  
  return best_projector;
}

VectorXd firstPrincipalComponent(const MatrixXd& instances)
{
  VectorXd mean = instances.rowwise().sum() / (double)instances.cols();
  MatrixXd centered(instances.rows(), instances.cols());
  for(int i = 0; i < centered.cols(); ++i)
    centered.col(i) = instances.col(i) - mean;

  MatrixXd xxt = centered * centered.transpose();

  // -- Use the power method.
  VectorXd evect = randomVector(centered.rows());
  evect.normalize();
  VectorXd prev;
  while(true) {
    prev = evect;
    evect = xxt * evect;
    if(evect.norm() == 0) {
      cout << "Had to reset..." << endl;
      randomVector(centered.rows());
    }
    
    evect.normalize();
    if((evect - prev).norm() < 1e-10)
      break;
  }

  return evect;
}


VectorXd randomVector(int num)
{
  VectorXd random(num);
  for(int i = 0; i < num; ++i)
    random(i) = (((double)rand() / (double)RAND_MAX) * 2.0) - 1.0;

  return random;
}

VectorXd createProjectionDirection(const VectorXd& pc)
{
  VectorXd projector = VectorXd::Zero(pc.rows());
  for(int i = 0; i < projector.rows(); ++i)
    projector(i) = copysign(1.0 / sqrt((double)pc.rows()), pc(i));
  return projector;
}

bool isDuplicateProjector(const Eigen::VectorXf& projector, const std::vector<Eigen::VectorXf>& others)
{
  bool dupe = false;
  for(size_t i = 0; i < others.size(); ++i) { 
    if((projector - others[i]).norm() < 1e-6) {
      dupe = true;
      break;
    }
  }
  return dupe;
}


