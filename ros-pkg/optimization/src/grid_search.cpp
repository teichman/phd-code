#include <optimization/grid_search.h>

using namespace std;
using namespace Eigen;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

GridSearch::GridSearch(int num_variables) :
  verbose_(true),
  max_resolutions_(num_variables),
  grid_radii_(num_variables),
  scale_factors_(num_variables),
  num_scalings_(-1)
{
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

  x_ = x;
  best_obj_ = numeric_limits<double>::max();
  res_ = max_resolutions_;

  int ns = 0;
  while(true) {
    bool improved = false;
    
    for(int i = 0; i < x_.rows(); ++i) {
      // -- Get all values of x to try in parallel.
      vector<ArrayXd> xs;  // TODO: reserve
      ArrayXd x = x_;
      for(int j = -grid_radii_[i]; j <= grid_radii_[i]; ++j)  {
	x[i] = x_[i] + j * res_[i];
	xs.push_back(x);
      }
    
      // -- Try them all in parallel.
      ArrayXd vals(xs.size());
      omp_set_num_threads(NUM_THREADS);
      #pragma omp parallel for
      for(size_t j = 0; j < xs.size(); ++j)
	vals(j) = objective_->eval(xs[j]);

      // -- Look for improvement.
      for(int j = 0; j < vals.rows(); ++j) {
	if(vals(j) < best_obj_) {
	  improved = true;
	  best_obj_ = vals(j);
	  x_ = xs[j];
	  if(verbose_)
	    cout << "*** ";
	  history_.push_back(x_);
	}

	if(verbose_)
	  cout << "obj = " << vals(j) << ", x = " << xs[j].transpose() << endl;
      }
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
