#ifndef POSE_GRAPH_SLAM_H
#define POSE_GRAPH_SLAM_H

#include <iostream>
#include <vector>
#include <ros/console.h>
#include <ros/assert.h>
#include <g2o/types/slam3d/vertex_se3_quat.h>
#include <g2o/types/slam3d/edge_se3_quat.h>
#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class PoseGraphSLAM
{
public:
  //! The first node is assumed to be at the origin with identity rotation.
  PoseGraphSLAM(int num_nodes);
  //! Edges encode translation then rotation for idx0 -> idx1.
  //! The covariance matrix orders the variables as translation, then rotation.
  //! covariance must be positive definite.
  void addEdge(int idx0, int idx1,
	       const Eigen::Vector3d& translation,
	       const Eigen::Matrix3d& rotation,
	       const Matrix6d& covariance);
  void solve(int num_iters = 10);
  //! After solving, get the final pose of node idx.
  Eigen::Affine3d pose(int idx) const;
  
  size_t numNodes() const { return optimizer_.vertices().size(); }
  size_t numEdges() const { return optimizer_.edges().size(); }
  
protected:
  g2o::SparseOptimizer optimizer_;
};

#endif // POSE_GRAPH_SLAM_H
