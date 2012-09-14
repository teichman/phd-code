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
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <eigen_extensions/eigen_extensions.h>
#include <serializable/serializable.h>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

//! Not used yet.  For serialization of PoseGraphSlam objects.
class PGSEdge : public Serializable
{
public:
  int idx0_;
  int idx1_;
  Eigen::Affine3d transform_;
  Matrix6d covariance_;

  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
};

class PoseGraphSlam
{
public:
  typedef boost::shared_ptr<PoseGraphSlam> Ptr;
  typedef boost::shared_ptr<const PoseGraphSlam> ConstPtr;
  
  //! The first node is assumed to be at the origin with identity rotation.
  PoseGraphSlam(int num_nodes);
  //! transform takes points seen by idx1 and puts them in the idx0 frame.
  //! Applied to (0, 0, 0, 0, 0, 0), this gives the pose of idx1 in the idx0 frame.
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
  //! Number of edges for vertex idx.
  size_t numEdges(size_t idx) const;
  
protected:
  g2o::SparseOptimizer optimizer_;
};

#endif // POSE_GRAPH_SLAM_H
