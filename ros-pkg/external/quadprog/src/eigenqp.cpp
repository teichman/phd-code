#include <quadprog/eigenqp.h>

using namespace QuadProgPP;


EigenQP::EigenQP(const Eigen::MatrixXd& G,
                 const Eigen::VectorXd& g0,
                 const Eigen::MatrixXd& CI,
                 const Eigen::VectorXd& ci0) :
  G_(G),
  g0_(g0),
  CI_(CI),
  ci0_(ci0)
{
  ROS_ASSERT(0);
  ROS_ASSERT(CE_.rows() == 0 && CE_.cols() == 0);
  ROS_ASSERT(ce0_.rows() == 0);
}

EigenQP::EigenQP(const Eigen::MatrixXd& G,
                 const Eigen::VectorXd& g0,
                 const Eigen::MatrixXd& CE,
                 const Eigen::VectorXd& ce0,
                 const Eigen::MatrixXd& CI,
                 const Eigen::VectorXd& ci0) :
  G_(G),
  g0_(g0),
  CE_(CE),
  ce0_(ce0),
  CI_(CI),
  ci0_(ci0)
{
}

Vector<double> EigenQP::eigToQP(const Eigen::VectorXd& eig)
{
  Vector<double> qp(eig.rows());
  for(int i = 0; i < eig.rows(); ++i)
    qp[i] = eig(i);

  return qp;
}

Matrix<double> EigenQP::eigToQP(const Eigen::MatrixXd& eig)
{
  Matrix<double> qp(eig.rows(), eig.cols());
  for(int i = 0; i < eig.rows(); ++i)
    for(int j = 0; j < eig.cols(); ++j) 
      qp[i][j] = eig(i, j);

  return qp;
}

Eigen::VectorXd EigenQP::qpToEig(const Vector<double>& qp)
{
  Eigen::VectorXd eig(qp.size());
  for(size_t i = 0; i < qp.size(); ++i)
    eig(i) = qp[i];
  return eig;
}

double EigenQP::solve(Eigen::VectorXd* x) const
{
  ROS_ASSERT(x->rows() == g0_.size());
  Vector<double> qpx = eigToQP(*x);
  Matrix<double> G = eigToQP(G_);
  Vector<double> g = eigToQP(g0_);

  for(int i = 0; i < G_.rows(); ++i)
    for(int j = 0; j < G_.cols(); ++j)
      ROS_ASSERT(G_(i, j) == G[i][j]);

  double val = solve_quadprog(G, g,
                              eigToQP(CE_), eigToQP(ce0_),
                              eigToQP(CI_), eigToQP(ci0_), qpx);
  *x = qpToEig(qpx);
  return val;
}

