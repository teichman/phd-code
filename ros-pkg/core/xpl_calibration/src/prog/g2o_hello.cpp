#include <iostream>
#include <map>
#include <vector>

#include <ros/console.h>
#include <ros/assert.h>

#include <g2o/types/slam3d/vertex_se3_quat.h>
#include <g2o/types/slam3d/edge_se3_quat.h>
#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

using namespace std;
using namespace g2o;

/*
 * Edges are translation, then rotation.
 *
 *
 */

int main(int argc, char** argv)
{
  
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  SparseOptimizer optimizer;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* solver = new SlamBlockSolver(&optimizer, linearSolver);
  optimizer.setSolver(solver);

  // -- Add nodes.
  int num_vertices = 4;
  for(int i = 0; i < num_vertices; ++i) { 
    VertexSE3* v = new VertexSE3;
    v->setId(i);
    optimizer.addVertex(v);
  }
  
  // -- Add edges.
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  Matrix6d information = Matrix6d::Identity();

  {
    Vector3d translation(0, 1, 0);
    Matrix3d rotation = Matrix3d::Identity();

    EdgeSE3* odometry = new EdgeSE3;
    odometry->vertices()[0] = optimizer.vertex(0);
    odometry->vertices()[1] = optimizer.vertex(1);
    SE3Quat se3(rotation, translation);
    odometry->setMeasurement(se3);
    odometry->setInverseMeasurement(se3.inverse());
    odometry->setInformation(information);
    optimizer.addEdge(odometry);
  }

  {
    Quaternion<double> rotation;
    rotation = AngleAxis<double>(M_PI / 4, Vector3d(1, 0, 0));
    Vector3d translation(0, 1, 0);

    EdgeSE3* odometry = new EdgeSE3;
    odometry->vertices()[0] = optimizer.vertex(1);
    odometry->vertices()[1] = optimizer.vertex(2);
    SE3Quat se3(rotation, translation);
    odometry->setMeasurement(se3);
    odometry->setInverseMeasurement(se3.inverse());
    odometry->setInformation(information);
    optimizer.addEdge(odometry);
  }

  {
    Quaternion<double> rotation;
    rotation = AngleAxis<double>(M_PI / 4, Vector3d(1, 0, 0));
    Vector3d translation(0, 1, 0);

    EdgeSE3* odometry = new EdgeSE3;
    odometry->vertices()[0] = optimizer.vertex(2);
    odometry->vertices()[1] = optimizer.vertex(3);
    SE3Quat se3(rotation, translation);
    odometry->setMeasurement(se3);
    odometry->setInverseMeasurement(se3.inverse());
    odometry->setInformation(information);
    optimizer.addEdge(odometry);
  }
  
  // -- Run the solver.
  VertexSE3* p0 = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
  ROS_ASSERT(p0);
  p0->setToOrigin();
  p0->setFixed(true);
  
  cout << "Optimizing" << endl;
  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cout << "done." << endl;

  cout << "Result:" << endl;
  for(int i = 0; i < num_vertices; ++i) { 
    cout << "Vertex " << i << endl;
    double data[7];
    optimizer.vertex(i)->getEstimateData(data);
    for(int j = 0; j < 7; ++j)
      cout << data[j] << " ";
    cout << endl;
  }

  return 0;
}
