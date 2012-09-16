#include <pose_graph_slam/pose_graph_slam.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
using namespace g2o;


PoseGraphSlam::PoseGraphSlam(int num_nodes)
{
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  typedef LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
   
  SlamLinearCholmodSolver* linear_solver = new SlamLinearCholmodSolver();
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
  // cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
  // cout << "Adding edge with transform: " << endl;
  // cout << transform.matrix() << endl << endl;

  Matrix3d rotation = transform.matrix().block(0, 0, 3, 3);
  Vector3d translation = transform.translation();
  SE3Quat se3(rotation, translation);
  
  // Quaterniond quat(rotation);
  // cout << "--------------------" << endl;
  // cout << rotation << endl << endl;
  // cout << quat.toRotationMatrix() << endl << endl;
  // Vector3d e0(1, 0, 0);
  // cout << e0.transpose() << "  -----  norm " << e0.norm() << endl;
  // cout << (rotation * e0).transpose() << "  -----  norm " << (rotation * e0).norm() << endl;
  // cout << (quat.toRotationMatrix() * e0).transpose() << "  -----  norm " << (quat.toRotationMatrix() * e0).norm() << endl;
  // ROS_ASSERT(fabs((rotation * e0).norm() - 1) < 1e-6);
  // ROS_ASSERT(fabs((quat.toRotationMatrix() * e0).norm() - 1) < 1e-6);
  // ROS_ASSERT((quat.toRotationMatrix() - rotation).norm() < 1e-6);
  
  // cout << "####################" << endl;
  // cout << transform.matrix() << endl << endl;
  // cout << ((Affine3d)se3.to_homogenious_matrix()).matrix() << endl << endl;
  // ROS_ASSERT((((Affine3d)se3.to_homogenious_matrix()).matrix() - transform.matrix()).norm() < 1e-6);
  EdgeStruct edge_struct;
  edge_struct.idx0 = idx0;
  edge_struct.idx1 = idx1;
  edge_struct.transform = transform;
  edge_struct.covariance = covariance;
  edges_.push_back(edge_struct); 

  EdgeSE3* edge = new EdgeSE3;
  edge->vertices()[0] = optimizer_.vertex(idx0);
  edge->vertices()[1] = optimizer_.vertex(idx1);
  
  edge->setMeasurement(se3);
  edge->setInverseMeasurement(se3.inverse());
  edge->setInformation(covariance.inverse());
  optimizer_.addEdge(edge);
  edge_ptrs_.push_back(edge);
}
  
void PoseGraphSlam::removeEdge(size_t idx)
{
  //Remove from g2o
  optimizer_.removeEdge(edge_ptrs_[idx]);
  //Remove from my own lists
  edges_.erase(edges_.begin()+idx);
  edge_ptrs_.erase(edge_ptrs_.begin()+idx);
}

void PoseGraphSlam::solve(int num_iters)
{
  //for(size_t i = 0; i < vertices_.size(); i++)
  //  optimizer_.addVertex(vertices_[i]);
  //for(size_t i = 0; i < edges_.size(); i++)
  //  optimizer_.addEdge(edges_[i]);
  VertexSE3* v0 = dynamic_cast<VertexSE3*>(optimizer_.vertex(0));
  ROS_ASSERT(v0);
  v0->setToOrigin();
  v0->setFixed(true);

  // Quaterniond quat;
  // quat.setIdentity();
  // SE3Quat se3(quat, Vector3d(0, 0, 0));
  // v0->setEstimate(se3);
  
  cout << "Optimizing..." << endl;
  //optimizer_.setVerbose(true);
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

size_t PoseGraphSlam::numEdges(size_t idx) const
{
  const VertexSE3* v = dynamic_cast<const VertexSE3*>(optimizer_.vertex(idx));
  ROS_ASSERT(v);
  return v->edges().size();
}

void PoseGraphSlam::serialize(std::ostream& out) const
{
  //Output num vertices
  out << numNodes() << endl;
  //Output num edges
  out << edges_.size() << endl;
  for(size_t i = 0; i < edges_.size(); i++)
  {
    out << edges_[i].idx0 << " " << edges_[i].idx1 << endl;
    eigen_extensions::serializeASCII(edges_[i].transform.matrix(), out);
    eigen_extensions::serializeASCII(edges_[i].covariance, out);
  }
}
void PoseGraphSlam::deserialize(std::istream& in)
{
  size_t n_vert; in >> n_vert;
  if(numNodes() != n_vert)
  {
    ROS_ASSERT(numNodes() == 0);
    for(int i = 0; i < n_vert; ++i) { 
      VertexSE3* v = new VertexSE3;
      v->setId(i);
      optimizer_.addVertex(v);
    }
  }
  size_t n_edges; in >> n_edges;
  for(size_t i = 0; i < n_edges; i++)
  {
    int idx0; in >> idx0;
    int idx1; in >> idx1;
    Eigen::Matrix4d m; eigen_extensions::deserializeASCII(in, &m);
    Eigen::Affine3d transform; transform.matrix() = m;
    Matrix6d covariance; eigen_extensions::deserializeASCII(in, &covariance);
    addEdge(idx0, idx1, transform, covariance);
  }
}
