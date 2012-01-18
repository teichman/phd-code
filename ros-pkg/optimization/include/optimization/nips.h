#ifndef NIPS_H
#define NIPS_H

#include <vector>
#include <optimization/optimization.h>

class NesterovInteriorPointSolver
{
public:
  NesterovInteriorPointSolver(ScalarFunction* objective,
			      VectorFunction* gradient,
			      double tol,
			      double alpha,
			      double beta,
			      int max_num_iters,
			      double initial_stepsize,
			      bool debug);
  void addConstraint(ScalarFunction* constraint,
		     VectorFunction* gradient);
  Eigen::VectorXd solve(const Eigen::VectorXd& init);
  bool feasible(const Eigen::VectorXd& x);
  double barrierObjective(const Eigen::VectorXd& x);
  Eigen::VectorXd barrierGradient(const Eigen::VectorXd& x);

protected:
  //! f_0(x)
  ScalarFunction* objective_;
  //! grad f_0(x)
  VectorFunction* gradient_;
  //! f_i(x) \leq 0
  std::vector<ScalarFunction*> constraints_;
  //! grad f_i(x)
  std::vector<VectorFunction*> grad_constraints_;
  double tol_;
  double alpha_;
  double beta_;
  int max_num_iters_;
  double initial_stepsize_;
  bool debug_;
  double mu_;

  Eigen::VectorXd solveInner(const Eigen::VectorXd& init,
			     int* num_steps);
  double backtracking(double t,
		      const Eigen::VectorXd& x,
		      const Eigen::VectorXd& grad,
		      const Eigen::VectorXd& direction,
		      double objective,
		      int* num_backtracks);

};

#endif // NIPS_H
