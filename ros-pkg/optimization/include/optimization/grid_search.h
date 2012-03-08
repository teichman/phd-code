#ifndef GRID_SEARCH_H
#define GRID_SEARCH_H

#include <optimization/optimization.h>
#include <pipeline/pipeline.h>

class GridSearchPod : public pipeline::Pod
{
  double result_;
  
  DECLARE_POD(GridSearchPod);
  GridSearchPod(std::string name) :
    Pod(name)
  {
  }
    
  void compute()
  {
    result_ = objective_->eval(x_);
  }

protected:
  ScalarFunction::Ptr objective_;
  Eigen::VectorXd x_;
};

class GridSearch
{
public:
  bool verbose_;
  ScalarFunction::Ptr objective_;
  Eigen::VectorXd ranges_;
  Eigen::VectorXd min_resolutions_;
  Eigen::VectorXd max_resolutions_;
  Eigen::VectorXd scale_multipliers_;
  std::vector< std::vector<int> > couplings_;
  std::vector<Eigen::VectorXd> history_;
  int max_passes_;

  GridSearch(int num_variables);
  Eigen::VectorXd solve(const Eigen::VectorXd& x);

protected:
  Eigen::VectorXd x_;
  Eigen::VectorXd best_x_;
  Eigen::VectorXd scales_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
  Eigen::VectorXd res_;
  Eigen::VectorXd lower_bounds_;
  Eigen::VectorXd upper_bounds_;
  double best_obj_;
};


#endif // GRID_SEARCH_H
