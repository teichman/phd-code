#include <pose_graph_slam/pose_graph_slam.h>

using namespace std;
using namespace g2o;

PoseGraphSLAM::PoseGraphSLAM(int num_nodes)
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

void PoseGraphSLAM::addEdge(int idx0, int idx1,
			    const Eigen::Vector3d& translation,
			    const Eigen::Matrix3d& rotation,
			    const Matrix6d& covariance)
{
  EdgeSE3* edge = new EdgeSE3;
  edge->vertices()[0] = optimizer_.vertex(idx0);
  edge->vertices()[1] = optimizer_.vertex(idx1);
  SE3Quat se3(rotation, translation);
  edge->setMeasurement(se3);
  edge->setInverseMeasurement(se3.inverse());
  edge->setInformation(covariance.inverse());
  optimizer_.addEdge(edge);
}

void PoseGraphSLAM::solve(int num_iters)
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

Eigen::Affine3d PoseGraphSLAM::transform(int idx) const
{
  double data[7];
  optimizer_.vertex(idx)->getEstimateData(data);
  Vector3d translation;  // offset
  translation << data[0], data[1], data[2];
  Quaterniond quat(data[6], data[3], data[4], data[5]);  // heading
  
  return Translation3d(translation) * Affine3d(quat.toRotationMatrix());
}

