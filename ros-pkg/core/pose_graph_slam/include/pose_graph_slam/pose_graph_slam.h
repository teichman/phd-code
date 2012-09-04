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

class PoseGraphSlam
{
public:
  typedef boost::shared_ptr<PoseGraphSlam> Ptr;
  typedef boost::shared_ptr<const PoseGraphSlam> ConstPtr;
  
  //! The first node is assumed to be at the origin with identity rotation.
  PoseGraphSlam(int num_nodes);
  //! Assuming idx0 is the origin, transform applied to the origin gives the pose of idx1 in idx0's frame.
  //! That is, the rotation component is the heading and the translation component is the position
  //! in the rotated frame.
  //! The covariance matrix orders the variables as translation, then rotation.
  //! covariance must be positive definite.
  void addEdge(int idx0, int idx1,
	       const Eigen::Affine3d& transform,
	       const Matrix6d& covariance);
  void solve(int num_iters = 10);
  //! Gets the transform that will send points from idx's local coordinate frame
  //! to the global one which the first node defines.
  //! global = transform(idx) * local
  //! transform rotates, then translates.
  Eigen::Affine3d transform(int idx) const;
  void vertexData(int idx, Eigen::Vector3d* translation, Eigen::Quaterniond* quat) const;
  
  size_t numNodes() const { return optimizer_.vertices().size(); }
  size_t numEdges() const { return optimizer_.edges().size(); }
  
protected:
  g2o::SparseOptimizer optimizer_;
};

#endif // POSE_GRAPH_SLAM_H
