#include <optimization/optimization.h>

using namespace std;
using namespace Eigen;

GridSearch::GridSearch(int num_variables) :
  ranges_(num_variables),
  min_resolutions_(num_variables),
  max_resolutions_(num_variables),
  scale_multipliers_(num_variables)
{
}

Eigen::VectorXd GridSearch::solve(const VectorXd& x)
{
  assert(objective_);
  assert(x.rows() == min_resolutions_.rows());
  assert(ranges_.rows() == min_resolutions_.rows());
  assert(ranges_.rows() == max_resolutions_.rows());
  assert(ranges_.rows() == scale_multipliers_.rows());
  assert(tol_ > 0);

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
  
  while(true) {
    double prev_best_obj = best_obj_;
    
    for(size_t i = 0; i < couplings_.size(); ++i) {
      for(size_t j = 0; j < couplings_[i].size(); ++j) {
	int k = couplings_[i][j];
	for(x_[k] = lb_[k]; x_[k] <= ub_[k]; x_[k] += res_[k]) {
	  cout << "Trying x_" << k << " = " << x_[k] << endl;
	  double val = objective_->eval(x_);
	  if(val < best_obj_) {
	    best_obj_ = val;
	    best_x_ = x_;
	    cout << "Best obj so far: " << best_obj_ << ", best x: " << x_.transpose() << endl;
	  }
	}
      }
      x_ = best_x_;
    }

    if(x_.rows() == 1 || prev_best_obj - best_obj_ < tol_) {
      bool done = true;
      for(int i = 0; i < res_.rows(); ++i)
	if(res_[i] > min_resolutions_[i])
	  done = false;
      if(done)
	break;
      
      res_ = res_.array() * scale_multipliers_.array();
      scales_ = scales_.array() * scale_multipliers_.array();
      lb_ = x_ - (scales_.array() * ranges_.array()).matrix();
      ub_ = x_ + (scales_.array() * ranges_.array()).matrix();

      lb_ = lb_.array().max(lower_bounds_.array()).matrix();
      ub_ = ub_.array().min(upper_bounds_.array()).matrix();
      
      cout << "lower: " << lb_.transpose() << ", upper: " << ub_.transpose() << endl;
    }
  }

  return x_;
}
  
