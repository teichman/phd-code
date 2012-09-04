#include <pose_graph_slam/pose_graph_slam.h>

using namespace std;
using namespace g2o;

PoseGraphSlam::PoseGraphSlam(int num_nodes)
{
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  
  SlamLinearSolver* linear_solver = new SlamLinearSolver();
  linear_solver->setBlockOrdering(false);
  SlamBlockSolver* solver = new SlamBlockSolver(&optimizer_, linear_solver);
  optimizer_.setSolver(solver);

  for(int i = 0; i < num_nodes; ++i) { 
    VertexSE3* v = new VertexSE3;
    v->setId(i);
    optimizer_.addVertex(v);
  }
}

void PoseGraphSlam::addEdge(int idx0, int idx1,
			    const Eigen::Affine3d& transform,
			    const Matrix6d& covariance)
{
  // Apparently I am using g2o incorrectly, or they have some weird conventions.
  Matrix3d rotation = transform.matrix().block(0, 0, 3, 3);
  Vector3d translation = transform.translation();
  translation = rotation * translation;
  SE3Quat se3(rotation, translation);
  //ROS_ASSERT((se3.to_homogenious_matrix() - transform.matrix()).norm() < 1e-6);
  
  EdgeSE3* edge = new EdgeSE3;
  edge->vertices()[0] = optimizer_.vertex(idx0);
  edge->vertices()[1] = optimizer_.vertex(idx1);
  
  edge->setMeasurement(se3);
  edge->setInverseMeasurement(se3.inverse());
  edge->setInformation(covariance.inverse());
  optimizer_.addEdge(edge);
}

void PoseGraphSlam::solve(int num_iters)
{
  VertexSE3* v0 = dynamic_cast<VertexSE3*>(optimizer_.vertex(0));
  ROS_ASSERT(v0);
  v0->setToOrigin();
  v0->setFixed(true);

  // Quaterniond quat;
  // quat.setIdentity();
  // SE3Quat se3(quat, Vector3d(0, 0, 0));
  // v0->setEstimate(se3);
  
  cout << "Optimizing..." << endl;
  optimizer_.setVerbose(true);
  optimizer_.initializeOptimization();
  optimizer_.optimize(num_iters);
  cout << "done." << endl;
}

Eigen::Affine3d PoseGraphSlam::transform(int idx) const
{
  // double data[7];
  // optimizer_.vertex(idx)->getEstimateData(data);
  // Vector3d translation;  // offset
  // translation << data[0], data[1], data[2];
  // Quaterniond quat(data[6], data[3], data[4], data[5]);  // heading
  // return Translation3d(translation) * Affine3d(quat.toRotationMatrix());

  const VertexSE3* v = dynamic_cast<const VertexSE3*>(optimizer_.vertex(idx));
  ROS_ASSERT(v);
  SE3Quat se3 = v->estimate();
  return Eigen::Affine3d(se3.to_homogenious_matrix());  // awesome.
}

void PoseGraphSlam::vertexData(int idx, Vector3d* translation, Quaterniond* quat) const
{
  double data[7];
  optimizer_.vertex(idx)->getEstimateData(data);
  *translation << data[0], data[1], data[2];
  *quat = Quaterniond(data[6], data[3], data[4], data[5]);
}
