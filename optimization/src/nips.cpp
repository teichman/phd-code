#include <optimization/nips.h>

using namespace std;
using namespace Eigen;

NesterovInteriorPointSolver::NesterovInteriorPointSolver(ScalarFunction* objective,
							 VectorFunction* gradient,
							 double tol,
							 double alpha,
							 double beta,
							 int max_num_iters,
							 double initial_stepsize,
							 bool debug) :
  
  objective_(objective),
  gradient_(gradient),
  tol_(tol),
  alpha_(alpha),
  beta_(beta),
  max_num_iters_(max_num_iters),
  initial_stepsize_(initial_stepsize),
  debug_(debug),
  mu_(1.0)
{
}

void NesterovInteriorPointSolver::addConstraint(ScalarFunction* constraint,
						VectorFunction* gradient)
{
  constraints_.push_back(constraint);
  grad_constraints_.push_back(gradient);
}


double NesterovInteriorPointSolver::barrierObjective(const Eigen::VectorXd& x)
{
  double obj = objective_->eval(x);
  for(size_t i = 0; i < constraints_.size(); ++i)
    obj -= mu_ * log(-constraints_[i]->eval(x));

  return obj;
}

Eigen::VectorXd NesterovInteriorPointSolver::barrierGradient(const Eigen::VectorXd& x)
{
  VectorXd grad = gradient_->eval(x);
  for(size_t i = 0; i < grad_constraints_.size(); ++i)
    grad -= mu_ * grad_constraints_[i]->eval(x) / constraints_[i]->eval(x);

  return grad;
}

double NesterovInteriorPointSolver::backtracking(double t,
						 const VectorXd& x,
						 const VectorXd& grad,
						 const VectorXd& direction,
						 double objective,
						 int* num_backtracks)
{
  assert(beta_ < 1 && beta_ > 0);
  assert(alpha_ > 0 && alpha_ <= 0.5);

  if(num_backtracks)
    *num_backtracks = 0;
  
  while(!feasible(x + t * direction) ||
	barrierObjective(x + t * direction) > objective + alpha_ * t * grad.dot(direction)) {
    if(debug_ > 1)
      cout << "Backtracking: " << barrierObjective(x + t*direction) << " > " << objective + alpha_ * t * grad.dot(direction) << endl;
    t *= beta_;
    
    if(num_backtracks)
      *num_backtracks += 1;
  }

  return t;
}

bool NesterovInteriorPointSolver::feasible(const VectorXd& x)
{
  for(size_t i = 0; i < constraints_.size(); ++i)
    if(constraints_[i]->eval(x) > 0)
      return false;

  return true;
}

VectorXd NesterovInteriorPointSolver::solve(const VectorXd& init)
{
  assert(feasible(init));

  mu_ = 1.0;
  VectorXd x = init;
  VectorXd x_prev;
  while(true) {
    cout << "Solving for mu = " << mu_ << endl;
    x_prev = x;
    int num_steps;
    x = solveInner(x, &num_steps);
    cout << "Solved for mu = " << mu_ << endl;
    cout << "Took " << num_steps << " steps." << endl;
    cout << "Objective: " << objective_->eval(x) << endl;
    cout << "Gradient norm: " << gradient_->eval(x).norm() << endl;
    cout << "x = " << x.transpose() << endl;

    if(mu_ < 1e-6 || (x - x_prev).norm() / x_prev.norm() < 1e-6) {
      break;
    }
    mu_ *= 0.5;
  }

  return x;
}

VectorXd NesterovInteriorPointSolver::solveInner(const VectorXd& init, int* num_steps)
{
  assert(feasible(init));
  
  VectorXd x = init;
  VectorXd y = init;
  VectorXd gradient_x, gradient_y;
  VectorXd x_prev;
  double min_objective = FLT_MAX;
  VectorXd best_x;
  double k = 1;
  *num_steps = 0;
  int num_backtracks = 0;
  double mult = 0.5;
  
  gradient_x = barrierGradient(x);
  if(debug_) {
    cout << "Step 0, gradient norm " << gradient_x.norm() << ", objective " << barrierObjective(x) << endl;
  }

  while(true) {
    ++*num_steps;
    x_prev = x;

    double objective_y = barrierObjective(y);
    if(objective_y < min_objective) {
      min_objective = objective_y;
      best_x = y;
    }

    // -- Compute the gradient and step using backtracking.
    gradient_y = barrierGradient(y);
    // Adaptive stepsize choice.
    // if(num_backtracks == 0)
    //   mult *= 2.0;
    // else if(num_backtracks > 2)
    //   mult *= 2.0 / 3.0;
    // double stepsize = backtracking(mult * initial_stepsize_, y, gradient_y, -gradient_y, objective_y, &num_backtracks);

    // Regular backtracking.
    double stepsize = backtracking(initial_stepsize_, y, gradient_y, -gradient_y, objective_y, &num_backtracks);

    
    // Fixed stepsize choice.
    // double beta = 0.5;
    // double stepsize = initial_stepsize_;
    // while(!feasible(y - stepsize * gradient_y))
    //   stepsize *= beta;
    // cout << "  y = " << y.transpose() << endl;
    // cout << "  Gradient at y = " << gradient_y.transpose() << endl;
    // cout << "  Using stepsize of " << stepsize << endl;
    
    x = y - stepsize * gradient_y;
    y = x + (k / (k + 3.0)) * (x - x_prev);
    //cout << "  y = " << y.transpose() << endl;
    // // If not, restart the algorithm.
    if(!feasible(y)) {
      cout << "  Infeasible y!" << endl;
      k = 1;
      y = x;
    }

    
    
    // -- Check to see how good this location is.
    double objective_x = barrierObjective(x);
    if(objective_x < min_objective) {
      min_objective = objective_x;
      best_x = x;
    }

    gradient_x = barrierGradient(x);
    double norm = gradient_x.norm();
    
    if(debug_) {
      //cout << "Step " << k << ", gradient norm " << norm << ", objective " << objective_x << endl;
      cout << "Nesterov Step " << *num_steps << ", gradient norm " << norm
	   << ", objective " << setprecision(12) << objective_x
	   << ", (x - x_prev).norm() " << setprecision(6) << (x - x_prev).norm()
	   << ", " << num_backtracks << " backtracks." << endl;

      assert(feasible(x));
      cout << "  x = " << x.transpose() << endl;
    }

    cout.flush();
    assert(!isnan(norm));
    assert(!isinf(norm));

    if(norm < tol_) {
      cout << "Breaking due to tolerance." << endl;
      break;
    }
     else if(max_num_iters_ > 0 && *num_steps == max_num_iters_) {
      cout << "Breaking because num_iters " << *num_steps << " = max_num_iters_ = " << max_num_iters_ << endl;
      break;
    }
    ++k;
  }

  if(debug_) {
    cout << "Solver complete, gradient norm " << gradient_x.norm() << " < tolerance " << tol_ << ", objective " << barrierObjective(x) << endl;
    cout << "Best x found has gradient norm " << setprecision(12) << barrierGradient(best_x).norm() << ", objective " << barrierObjective(best_x) << endl;
  }

  return best_x;  
}

