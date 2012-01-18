#ifndef EIGENQP_H
#define EIGENQP_H

#include <Eigen/Eigen>
#include <ros/assert.h>
#include <quadprog/QuadProg.h>

//! See QuadProg.h.
//! min 1/2 x^T G x + g0^T x
class EigenQP
{
public:
  EigenQP(const Eigen::MatrixXd& G,
	  const Eigen::VectorXd& g0,
	  const Eigen::MatrixXd& CI,
	  const Eigen::VectorXd& ci0);
  
  EigenQP(const Eigen::MatrixXd& G,
	  const Eigen::VectorXd& g0,
	  const Eigen::MatrixXd& CE,
	  const Eigen::VectorXd& ce0,
	  const Eigen::MatrixXd& CI,
	  const Eigen::VectorXd& ci0);

  //! Pass in x as the starting point; it will be filled
  //! with the solution.
  double solve(Eigen::VectorXd* x) const;

  static QuadProgPP::Vector<double> eigToQP(const Eigen::VectorXd& eig);
  static QuadProgPP::Matrix<double> eigToQP(const Eigen::MatrixXd& eig);
  static Eigen::VectorXd qpToEig(const QuadProgPP::Vector<double>& qp);

protected:
  Eigen::MatrixXd G_;
  Eigen::VectorXd g0_;
  Eigen::MatrixXd CE_;
  Eigen::VectorXd ce0_;
  Eigen::MatrixXd CI_;
  Eigen::VectorXd ci0_;
};

#endif // EIGENQP_H
