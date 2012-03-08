#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cfloat>
#include <sys/time.h>


/************************************************************
 * Function representation
 ************************************************************/

template <typename Domain, typename Codomain> class Function
{
 public:
  typedef boost::shared_ptr<Function> Ptr;
  typedef boost::shared_ptr<const Function> ConstPtr;
  
  virtual Codomain eval(const Domain& x) = 0;
  Codomain operator()(const Domain& x) {return eval(x);}
};

typedef Function<Eigen::VectorXd, double> ScalarFunction;
typedef Function<Eigen::VectorXd, Eigen::VectorXd> VectorFunction;
typedef Function<Eigen::VectorXd, Eigen::MatrixXd> MatrixFunction;


/************************************************************
 * Unconstrained solvers
 ************************************************************/

class BisectionSolver
{
public:
  ScalarFunction* objective_;
  VectorFunction* gradient_;
  double tol_;
  double min_;
  double max_;
  int max_num_iters_;
  bool debug_;
  
  BisectionSolver(ScalarFunction* objective, VectorFunction* gradient,
		  double tol, double min, double max,
		  int max_num_iters = 0, bool debug = false);

  double solve();
};

class GridSearch
{
public:
  ScalarFunction::Ptr objective_;
  Eigen::VectorXd ranges_;
  Eigen::VectorXd min_resolutions_;
  Eigen::VectorXd max_resolutions_;
  Eigen::VectorXd scale_multipliers_;
  std::vector< std::vector<int> > couplings_;
  std::vector<Eigen::VectorXd> history_;

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

class NesterovGradientSolver {
 public:
  ScalarFunction* objective_;
  VectorFunction* gradient_;
  //! If this is set, then the condition number will be evaluated at each step.
  //! Like the other pointer members, this is not deleted.
  MatrixFunction* hessian_;
  double tol_;
  double alpha_;
  double beta_;
  int max_num_iters_;
  double initial_stepsize_;
  bool debug_;

  NesterovGradientSolver(ScalarFunction* objective,
			 VectorFunction* gradient,
			 double tol,
			 double alpha,
			 double beta,
			 int max_num_iters = 0,
			 double initial_stepsize = 1,
			 bool debug = false);
  Eigen::VectorXd solve(const Eigen::VectorXd& init);

 private:
  double backtracking(double t, const Eigen::VectorXd& x,
		      const Eigen::VectorXd& grad,
		      const Eigen::VectorXd& direction,
		      double objective, int* num_backtracks);
};
  
class NewtonSolver {
 public:
  ScalarFunction* objective_;
  VectorFunction* gradient_;
  MatrixFunction* hessian_;
  double tol_;
  double alpha_;
  double beta_;
  double stepsize_;
  int debug_;
  int max_iters_;

  NewtonSolver(ScalarFunction* objective,
	       VectorFunction* gradient,
	       MatrixFunction* hessian,
	       double tol,
	       double alpha,
	       double beta,
	       double stepsize,
	       int debug = 0,
	       int max_iters = 0);
  
  Eigen::VectorXd solve(Eigen::VectorXd x);
  std::string progressString(int k, const Eigen::VectorXd& grad,
			     const Eigen::VectorXd& x, double lambda2,
			     double objective, int num_backtracks);
  
 private:
  double backtracking(double t, const Eigen::VectorXd& x,
		      const Eigen::VectorXd& grad,
		      const Eigen::VectorXd& direction,
		      double objective, int* num_backtracks = NULL);
};

class GradientSolver
{
public:
  ScalarFunction* objective_;
  VectorFunction* gradient_;
  double tol_;
  double alpha_;
  double beta_;
  int max_num_iters_;
  double initial_stepsize_;
  bool debug_;

  GradientSolver(ScalarFunction* objective,
		 VectorFunction* gradient,
		 double tol,
		 double alpha,
		 double beta,
		 int max_num_iters = 0,
		 double initial_stepsize = 1,
		 bool debug = false);
  Eigen::VectorXd solve(const Eigen::VectorXd& init);

private:
  double backtracking(double t, const Eigen::VectorXd& x,
		      const Eigen::VectorXd& grad,
		      const Eigen::VectorXd& direction,
		      double objective, int* num_backtracks);
};

#endif // OPTIMIZATION_H
