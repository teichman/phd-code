
#include <optimization/optimization.h>
#include <optimization/common_functions.h>
#include <optimization/nips.h>
#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;    

void generateRandomProblemData(MatrixXd* A, VectorXd* b,
			       Eigen::SparseMatrix<double, Eigen::ColMajor>* A_sparse)
{
  // -- Problem setup.
  int num_vars = 10;
  int num_tr_ex = 1000;
  *b = VectorXd::Random(num_tr_ex);
  
  // -- Set up dense version of A.
  *A = MatrixXd::Zero(num_vars, num_tr_ex);
  int num_nonzero = 0;
  for(int i = 0; i < A->rows(); ++i) {
    for(int j = 0; j < A->cols(); ++j) {
      double val = (double)rand() / (double)RAND_MAX;
      if(val > 0.9) {
	A->coeffRef(i, j) = val;
	++num_nonzero;
      }
    }
  }
  
  // -- Set up sparse version of A.
  *A_sparse = Eigen::SparseMatrix<double, Eigen::ColMajor>(A->rows(), A->cols());
//   cout << "A: " << A.rows() << " x " << A.cols() << endl;
//   cout << "A_sparse: " << A_sparse.rows() << " x " << A_sparse.cols() << endl;

  A_sparse->startFill(num_nonzero * 1.1);
  int filled = 0;
  for(int j = 0; j < A->cols(); ++j) {
    for(int i = 0; i < A->rows(); ++i) {
      if(A->coeff(i, j) != 0) { 
	A_sparse->fill(i, j) = A->coeff(i, j);
	++filled;
      }
    }
  }
  A_sparse->endFill();
  assert(filled == num_nonzero);
}

// A must be symmetric.
class QuadraticFunction : public ScalarFunction
{
public:
  QuadraticFunction(const Eigen::MatrixXd& A,
		    const Eigen::VectorXd& b,
		    double c) :
    ScalarFunction(),
    A_(A),
    b_(b),
    c_(c)
    {
    }
  
  double eval(const Eigen::VectorXd& x) const
    {
      return 0.5 * x.transpose() * A_ * x + b_.dot(x) + c_;
    }

protected:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  double c_;
};

class QuadraticGradient : public VectorFunction
{
public:
  QuadraticGradient(const Eigen::MatrixXd& A,
		    const Eigen::VectorXd& b) :
    VectorFunction(),
    A_(A),
    b_(b)
    {
    }
  
  VectorXd eval(const Eigen::VectorXd& x) const
    {
      return A_ * x + b_;
    }

protected:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
};

class QuadraticHessian : public MatrixFunction
{
public:
  QuadraticHessian(const Eigen::MatrixXd& A) :
    MatrixFunction(),
    A_(A)
    {
    }

  MatrixXd eval(const Eigen::VectorXd& x) const
    {
      return A_;
    }

protected:
  Eigen::MatrixXd A_;
};

// TEST(Nesterov, SimpleProblem)
// {
//   MatrixXd A = MatrixXd::Identity(2, 2);
//   A(0, 0) = 1000;
//   VectorXd b = VectorXd::Zero(2);
//   QuadraticFunction obj(A, b, 0);
//   QuadraticGradient grad(A, b);
//   double tol = 1e-6;
//   double alpha = 0.3;
//   double beta = 0.5;
//   int max_num_iters = 0;
//   double stepsize = 1.0;
//   NesterovGradientSolver ngs(&obj, &grad, tol, alpha, beta, max_num_iters, stepsize, true);
//   VectorXd solution = ngs.solve(VectorXd::Ones(2) * 100);
//   cout << "Got solution:" << endl << solution.transpose() << endl;
// }

// TEST(NesterovInteriorPointSolver, SimpleProblem)
// {
//   MatrixXd A = MatrixXd::Identity(2, 2);
//   A(0, 0) = 100;
//   VectorXd b = VectorXd::Zero(2);
//   QuadraticFunction obj(A, b, 0);
//   QuadraticGradient grad(A, b);
//   double tol = 1e-6;
//   double alpha = 0.3;
//   double beta = 0.5;
//   int max_num_iters = 0;
//   double stepsize = 1e-4;
//   NesterovInteriorPointSolver nips(&obj, &grad, tol, alpha, beta, max_num_iters, stepsize, true);
//   VectorXd nips_soln = nips.solve(VectorXd::Ones(2) * 100);
//   cout << "NIPS solution:" << endl << nips_soln.transpose() << endl;

//   NesterovGradientSolver ngs(&obj, &grad, tol, alpha, beta, max_num_iters, stepsize, false);
//   VectorXd ngs_soln = ngs.solve(VectorXd::Ones(2) * 100);
//   cout << "NGS solution:" << endl << ngs_soln.transpose() << endl;
// }

TEST(NesterovInteriorPointSolver, SimpleConstrainedProblem)
{
  MatrixXd A = MatrixXd::Identity(2, 2);
  A(0, 0) = 1;
  VectorXd b = VectorXd::Zero(2);
  QuadraticFunction obj(A, b, 0);
  QuadraticGradient grad(A, b);
  double tol = 1e-6;
  double alpha = 0.3;
  double beta = 0.5;
  int max_num_iters = 0;
  double stepsize = 1e-3;
  NesterovInteriorPointSolver nips(&obj, &grad, tol, alpha, beta, max_num_iters, stepsize, true);

  // -- x0 \geq 2
  VectorXd b0 = VectorXd::Zero(2);
  b0(0) = -1;
  QuadraticFunction c0(MatrixXd::Zero(2, 2), b0, 2);
  QuadraticGradient gc0(MatrixXd::Zero(2, 2), b0);
  nips.addConstraint(&c0, &gc0);

  // -- x1 \leq 100
  VectorXd b1 = VectorXd::Zero(2);
  b1(1) = 1;
  QuadraticFunction c1(MatrixXd::Zero(2, 2), b1, -100);
  QuadraticGradient gc1(MatrixXd::Zero(2, 2), b1);
  nips.addConstraint(&c1, &gc1);

  // -- x1 \geq 5
  VectorXd b2 = VectorXd::Zero(2);
  b2(1) = -1;
  QuadraticFunction c2(MatrixXd::Zero(2, 2), b2, 5);
  QuadraticGradient gc2(MatrixXd::Zero(2, 2), b2);
  nips.addConstraint(&c2, &gc2);

  // -- unit ball constraint at x = (10, 10).
  QuadraticFunction c3(2.0 * MatrixXd::Identity(2, 2), -20 * VectorXd::Ones(2), 199);
  QuadraticGradient gc3(2.0 * MatrixXd::Identity(2, 2), -20 * VectorXd::Ones(2));
  nips.addConstraint(&c3, &gc3);
  
  VectorXd init(2);
  init(0) = 10;
  init(1) = 10;
  VectorXd nips_soln = nips.solve(init);
  cout << "NIPS solution:" << endl << nips_soln.transpose() << endl;
}


// TODO: This fails.  What happened?
// TEST(SparseMLS, correctness)
// {

// //   cout << "A: " << endl << A << endl;
// //   cout << "A_sparse: " << endl << A_sparse << endl;
//   MatrixXd A;
//   VectorXd b;
//   Eigen::SparseMatrix<double, Eigen::ColMajor> A_sparse;
//   generateRandomProblemData(&A, &b, &A_sparse);

//   // -- Get mean logistic score for a random vector using both methods.
//   VectorXd x = VectorXd::Random(A.rows());
//   ObjectiveMLS obj_mls(A, b);
//   double regular = obj_mls.eval(x);

//   ObjectiveMLSSparse obj_mls_sparse(&A_sparse, &b);
//   double sparse = obj_mls_sparse.eval(x);

//   cout << "Delta: " << regular - sparse << endl;
//   EXPECT_DOUBLE_EQ(regular, sparse);

//   // -- Get gradient with both methods.
//   GradientMLS grad_mls(A, b);
//   VectorXd grad = grad_mls.eval(x);
//   GradientMLSSparse grad_mls_sparse(&A_sparse, &b);
//   VectorXd grad_sparse = grad_mls_sparse.eval(x);
//   cout << grad.transpose() << endl << grad_sparse.transpose() << endl;
//   cout << "Norm of the gradient difference: " << (grad - grad_sparse).norm() << endl;
//   for(int i = 0; i < grad.rows(); ++i)
//     EXPECT_DOUBLE_EQ(grad(i), grad_sparse(i));

//   // -- Get Hessian with both methods.
//   HessianMLS hess_mls(A, b);
//   MatrixXd hess = hess_mls.eval(x);
//   HessianMLSSparse hess_mls_sparse(&A_sparse, &b);
//   MatrixXd hess_sparse = hess_mls_sparse.eval(x);
//   double fn = 0;
//   for(int i = 0; i < hess.rows(); ++i) { 
//     for(int j = 0; j < hess.cols(); ++j) { 
//       EXPECT_DOUBLE_EQ(hess(i, j), hess_sparse(i, j));
//       fn += pow(hess(i, j) - hess_sparse(i, j), 2);
//     }
//   }
//   cout << "Frobenius norm of hessian difference: " << sqrt(fn) << endl;
// //   cout << hess << endl;
// //   cout << hess_sparse << endl;
    
// }

//TODO: Re-enable this.
// TEST(UnconstrainedOptimization, SparseMEL_matches_MEL)
// {
//   MatrixXd A;
//   VectorXd b;
//   Eigen::SparseMatrix<double, Eigen::ColMajor> A_sparse;
//   generateRandomProblemData(&A, &b, &A_sparse);

//   for(int j = 0; j < 10; ++j) { 
//     // -- Get mean exponential loss for a random vector using both methods.
//     VectorXd x = VectorXd::Random(A.rows());
//     ObjectiveMEL obj_mel(A, b);
//     double regular = obj_mel.eval(x);
    
//     ObjectiveMELSparse obj_mel_sparse(&A_sparse, &b);
//     double sparse = obj_mel_sparse.eval(x);
    
//     cout << "Delta: " << regular - sparse << endl;
//     EXPECT_FLOAT_EQ(regular, sparse);

//     // -- Get gradient with both methods.
//     GradientMEL grad_mel(A, b);
//     VectorXd grad = grad_mel.eval(x);
//     GradientMELSparse grad_mel_sparse(&A_sparse, &b);
//     VectorXd grad_sparse = grad_mel_sparse.eval(x);
//     cout << grad.transpose() << endl << grad_sparse.transpose() << endl;
//     cout << "Norm of the gradient difference: " << (grad - grad_sparse).norm() << endl;
//     for(int i = 0; i < grad.rows(); ++i)
//       EXPECT_FLOAT_EQ(grad(i), grad_sparse(i));
//   }
  
// //   // -- Get Hessian with both methods.
// //   HessianMEL hess_mel(A, b);
// //   MatrixXd hess = hess_mel.eval(x);
// //   HessianMELSparse hess_mel_sparse(&A_sparse, &b);
// //   MatrixXd hess_sparse = hess_mel_sparse.eval(x);
// //   double fn = 0;
// //   for(int i = 0; i < hess.rows(); ++i) { 
// //     for(int j = 0; j < hess.cols(); ++j) { 
// //       EXPECT_DOUBLE_EQ(hess(i, j), hess_sparse(i, j));
// //       fn += pow(hess(i, j) - hess_sparse(i, j), 2);
// //     }
// //   }
// //   cout << "Frobenius norm of hessian difference: " << sqrt(fn) << endl;

  
// }

// TODO: this fails.  What happened?
// TEST(UnconstrainedOptimization, NewtonSolver)
// {
//   int num_vars = 10;
//   int num_tr_ex = 1000;
//   MatrixXd A = MatrixXd::Random(num_vars, num_tr_ex);
//   VectorXd b = VectorXd::Random(num_tr_ex);

//   ObjectiveMLS obj_mls(A, b);
//   GradientMLS grad_mls(A, b);
//   HessianMLS hess_mls(A, b);

//   cout << "Solving." << endl;
//   VectorXd init = VectorXd::Zero(A.rows());
//   double tol = 1e-6;
//   double alpha = 0.3;
//   double beta = 0.5;
//   double stepsize = 1.0;
//   NewtonSolver solver_mls(&obj_mls, &grad_mls, &hess_mls, tol, alpha, beta, stepsize, true);
//   VectorXd solution = solver_mls.solve(init);
//   cout << "Got solution:" << endl << solution.transpose() << endl;

//   cout << "Perturbing..." << endl;
//   for(int i = 0; i < 1e4; ++i) {
//     VectorXd random = VectorXd::Random(solution.rows());
//     random *= ((double)rand() / (double)RAND_MAX);
//     VectorXd perturbed = solution + random;

//     EXPECT_TRUE(obj_mls.eval(perturbed) + 1e-6 >= obj_mls.eval(solution));
//   }

//   cout << "Solving with Nesterov." << endl;
//   NesterovGradientSolver ngs(&obj_mls, &grad_mls, tol, alpha, beta, 0, 1, true);
//   ngs.hessian_ = &hess_mls; // Compute condition number.
//   VectorXd nesterov_solution = ngs.solve(init);
//   cout << "Got solution:" << endl << nesterov_solution.transpose() << endl;
  
//   cout << "Perturbing..." << endl;
//   for(int i = 0; i < 1e4; ++i) {
//     VectorXd random = VectorXd::Random(nesterov_solution.rows());
//     random *= ((double)rand() / (double)RAND_MAX);
//     VectorXd perturbed = nesterov_solution + random;

//     EXPECT_TRUE(obj_mls.eval(perturbed) + 1e-6 >= obj_mls.eval(solution));
//   }


//   cout << "Solving with Gradient." << endl;
//   GradientSolver gs(&obj_mls, &grad_mls, tol, alpha, beta, 0, 1, true);
//   VectorXd gradient_solution = gs.solve(init);
//   cout << "Got solution:" << endl << gradient_solution.transpose() << endl;

//   cout << "Perturbing..." << endl;
//   double gradient_objective = obj_mls.eval(gradient_solution);
//   for(int i = 0; i < 1e4; ++i) {
//     VectorXd random = VectorXd::Random(gradient_solution.rows());
//     random *= ((double)rand() / (double)RAND_MAX);
//     VectorXd perturbed = gradient_solution + random;
//     EXPECT_TRUE(obj_mls.eval(perturbed) >= gradient_objective);
//   }
// }


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
