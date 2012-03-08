#include <optimization/grid_search.h>

using namespace std;
using namespace Eigen;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

GridSearch::GridSearch(int num_variables) :
  verbose_(true),
  ranges_(num_variables),
  min_resolutions_(num_variables),
  max_resolutions_(num_variables),
  scale_multipliers_(num_variables),
  max_passes_(0)
{
}

Eigen::VectorXd GridSearch::solve(const VectorXd& x)
{
  history_.clear();
  assert(objective_);
  assert(x.rows() == min_resolutions_.rows());
  assert(ranges_.rows() == min_resolutions_.rows());
  assert(ranges_.rows() == max_resolutions_.rows());
  assert(ranges_.rows() == scale_multipliers_.rows());

  for(int i = 0; i < scale_multipliers_.rows(); ++i)
    assert(scale_multipliers_[i] > 0 && scale_multipliers_[i] < 1);
  
  if(couplings_.empty()) {
    couplings_.resize(x.rows());
    for(int i = 0; i < x.rows(); ++i)
      couplings_[i].push_back(i);
  }

  x_ = x;
  best_obj_ = numeric_limits<double>::max();
  scales_ = VectorXd::Ones(x_.rows());
  res_ = max_resolutions_;
  lb_ = x_ - (scales_.array() * ranges_.array()).matrix();
  ub_ = x_ + (scales_.array() * ranges_.array()).matrix();
  lower_bounds_ = lb_;
  upper_bounds_ = ub_;

  int pass = 0;
  while(true) {
    bool improved = false;
    
    for(size_t i = 0; i < couplings_.size(); ++i) {
      for(size_t j = 0; j < couplings_[i].size(); ++j) {
	int k = couplings_[i][j];
	VectorXd x = x_;
		
	vector<VectorXd> xs;
	for(x[k] = lb_[k]; x[k] <= ub_[k]; x[k] += res_[k])
	  xs.push_back(x);
	VectorXd vals(xs.size());

	omp_set_num_threads(NUM_THREADS);
        #pragma omp parallel for
	for(size_t l = 0; l < xs.size(); ++l) { 
	  vals(l) = objective_->eval(xs[l]);
	}
	
	for(int l = 0; l < vals.rows(); ++l) {
	  if(verbose_)
	    cout << "x[" << k << "] = " << xs[l](k) << ": " << vals(l) << endl;
	  if(vals(l) < best_obj_) {
	    improved = true;
	    best_obj_ = vals(l);
	    best_x_ = xs[l];
	    if(verbose_)
	      cout << "** Best obj so far: " << best_obj_ << ", best x: " << x.transpose() << endl;
	    history_.push_back(x);
	  }
	}
      }
      x_ = best_x_;
    }

    ++pass;
    if(x_.rows() == 1 || pass == max_passes_ || !improved) {
      if(verbose_)
	cout << "Scaling down." << endl;
      bool done = true;
      for(int i = 0; i < res_.rows(); ++i)
	if(res_[i] - min_resolutions_[i] > 1e-6)
	  done = false;
      if(done)
	break;
    }

    res_ = res_.array() * scale_multipliers_.array();
    scales_ = scales_.array() * scale_multipliers_.array();
    lb_ = x_ - (scales_.array() * ranges_.array()).matrix();
    ub_ = x_ + (scales_.array() * ranges_.array()).matrix();
    
    lb_ = lb_.array().max(lower_bounds_.array()).matrix();
    ub_ = ub_.array().min(upper_bounds_.array()).matrix();

    if(verbose_)
      cout << "lower: " << lb_.transpose() << ", upper: " << ub_.transpose() << endl;
  }
  
  return x_;
}
  


// void* search(void* gridsearch)
// {
//   GridSearch& gs = *((GridSearch*) gridsearch);
//   gs.objective_->eval(
//   pl.run();
//   return NULL;
// }
