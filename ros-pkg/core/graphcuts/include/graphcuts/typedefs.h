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
  typedef boost::shared_ptr<SparseMat> SparseMatPtr;
  typedef boost::shared_ptr<const SparseMat> SparseMatConstPtr;

  // Useful for creating SparseMat in a more programmer-friendly way.
  typedef Eigen::DynamicSparseMatrix<double, Eigen::RowMajor> DynamicSparseMat;
  typedef boost::shared_ptr<DynamicSparseMat> DynamicSparseMatPtr;
  typedef boost::shared_ptr<const DynamicSparseMat> DynamicSparseMatConstPtr;
  
  // Labels are 0 and 1.  TODO: This should probably be {-1, 0, +1} instead.
  typedef Eigen::VectorXi VecXi;
  typedef boost::shared_ptr<Eigen::VectorXi> VecXiPtr;
  typedef boost::shared_ptr<const Eigen::VectorXi> VecXiConstPtr;
  
}

#endif // GRAPHCUTS_TYPEDEFS_H
