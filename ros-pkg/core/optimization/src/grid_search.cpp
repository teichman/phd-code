#include <optimization/grid_search.h>

using namespace std;
using namespace Eigen;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

GridSearch::GridSearch(int num_variables) :
  verbose_(true),
  num_scalings_(-1),
  max_resolutions_(num_variables),
  grid_radii_(num_variables),
  scale_factors_(num_variables),
  couplings_(num_variables),
  view_handler_(NULL)
{
  for(int i = 0; i < num_variables; ++i)
    couplings_[i] = i;
}

void GridSearch::appendVariations(int id, const Eigen::ArrayXd& orig,
				  std::vector<Eigen::ArrayXd>* xs) const
{
  for(int i = -grid_radii_[id]; i <= grid_radii_[id]; ++i)  {
    if(i == 0)
      continue;
    ArrayXd x = orig;
    x[id] = x[id] + i * res_[id];
    xs->push_back(x);
  }
}

void GridSearch::makeGrid(const std::vector<int>& variables,
			  std::vector<Eigen::ArrayXd>* xs) const
{
  xs->clear();
  xs->push_back(x_);
  vector<ArrayXd> xs2;
  for(size_t i = 0; i < variables.size(); ++i) {
    xs2.clear();
    for(size_t j = 0; j < xs->size(); ++j)
      appendVariations(variables[i], xs->at(j), &xs2);
    xs->insert(xs->end(), xs2.begin(), xs2.end());
  }

  if(verbose_) {
    cout << "** Grid ** " << endl;
    cout << "Variables: " << endl;
    for(size_t i = 0; i < variables.size(); ++i)
      cout << variables[i] << " ";
    cout << endl;
    cout << "Xs: " << endl;
    for(size_t i = 0; i < xs->size(); ++i)
      cout << xs->at(i).transpose() << endl;
  }
}

Eigen::ArrayXd GridSearch::search(const ArrayXd& x)
{
  assert(num_scalings_ >= 0);
  assert(objective_);
  assert(x.rows() == max_resolutions_.rows());
  assert(x.rows() == grid_radii_.rows());
  assert(x.rows() == scale_factors_.rows());
  for(int i = 0; i < scale_factors_.rows(); ++i)
    assert(scale_factors_[i] > 0 && scale_factors_[i] < 1);
  history_.clear();

  vector< vector<int> > couplings;
  for(int i = 0; i < x.rows(); ++i) {
    vector<int> coup;
    for(int j = 0; j < couplings_.rows(); ++j)
      if(couplings_(j) == i)
	coup.push_back(j);
    if(!coup.empty())
      couplings.push_back(coup);
  }
  
  x_ = x;
  best_obj_ = numeric_limits<double>::max();
  res_ = max_resolutions_;

  int ns = 0;
  while(true) {
    bool improved = false;

    vector<ArrayXd> xs;
    for(size_t i = 0; i < couplings.size(); ++i) {
      // -- Get all values of x to try in parallel.
      vector<ArrayXd> xs_this_coupling;
      makeGrid(couplings[i], &xs_this_coupling);
      xs.insert(xs.end(), xs_this_coupling.begin(), xs_this_coupling.end());
    }

    // -- Try them all in parallel.
    ArrayXd vals(xs.size());
    omp_set_num_threads(NUM_THREADS);
#pragma omp parallel for
    for(size_t j = 0; j < xs.size(); ++j) { 
      vals(j) = objective_->eval(xs[j]);
      assert(!isnan(vals(j)));
    }
    
    // -- Look for improvement.
    for(int j = 0; j < vals.rows(); ++j) {
      if(vals(j) < best_obj_) {
	improved = true;
	best_obj_ = vals(j);
	x_ = xs[j];
	if(verbose_)
	  cout << "*** ";
	if(view_handler_)
	  view_handler_->handleGridSearchUpdate(xs[j], vals(j));
	
	history_.push_back(x_);
      }
      
      if(verbose_)
	cout << "obj = " << vals(j) << ", x = " << xs[j].transpose() << endl;
    }
      
    if(improved)
      continue;
    if(ns == num_scalings_)
      break;
    ++ns;
    res_ *= scale_factors_;
    if(verbose_)
      cout << "Using step sizes of " << res_.transpose() << endl;
  }
  
  return x_;
}
