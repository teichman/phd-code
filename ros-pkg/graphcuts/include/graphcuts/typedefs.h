#ifndef GRAPHCUTS_TYPEDEFS_H
#define GRAPHCUTS_TYPEDEFS_H

#include <boost/shared_ptr.hpp>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <maxflow/graph.h>

namespace graphcuts
{

  typedef Graph<double, double, double> Graph3d;
  typedef boost::shared_ptr<Graph3d> Graph3dPtr;


  typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SparseMat;
  typedef boost::shared_ptr< Eigen::SparseMatrix<double, Eigen::RowMajor> > SparseMatPtr;
  typedef boost::shared_ptr< const Eigen::SparseMatrix<double, Eigen::RowMajor> > SparseMatConstPtr;

  // Labels are 0 and 1.
  typedef Eigen::VectorXi VecXi;
  typedef boost::shared_ptr<Eigen::VectorXi> VecXiPtr;
  typedef boost::shared_ptr<const Eigen::VectorXi> VecXiConstPtr;
  
}

#endif // GRAPHCUTS_TYPEDEFS_H
