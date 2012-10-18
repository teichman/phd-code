#include <pose_graph_slam/pose_graph_slam.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
using namespace g2o;


PoseGraphSlam::PoseGraphSlam(size_t num_nodes)
{
  initialize(num_nodes);
}

void
PoseGraphSlam::initialize(size_t num_nodes)
{
  nodes_.resize(num_nodes);
  for(size_t i = 0; i < num_nodes; i++)
    nodes_[i] = (int)i;
}

void PoseGraphSlam::addEdge(int idx0, int idx1,
			    const Eigen::Affine3d& transform,
			    const Matrix6d& covariance)
{
  EdgeStruct edge_struct;
  edge_struct.idx0 = idx0;
  edge_struct.idx1 = idx1;
  edge_struct.transform = transform;
  edge_struct.covariance = covariance;
  edges_.push_back(edge_struct); 
  update_graph_ = true;
  update_subgraphs_ = true;
}
  
void PoseGraphSlam::removeEdge(size_t idx)
{
  edges_.erase(edges_.begin()+idx);
  update_graph_ = true;
  update_subgraphs_ = true;
}

size_t PoseGraphSlam::solve(size_t min_size, int num_iters)
{
  ROS_ASSERT(min_size > 1);
  vector<Graph_t::Ptr> subgraphs; getSubgraphs(subgraphs, min_size);
  optimizers_cache_.resize(subgraphs.size());
  for(size_t i = 0; i <  subgraphs.size(); i++)
  {
    Graph_t::Ptr subgraph = subgraphs[i];
    vector<int> nodes(subgraph->nodes_.size());
    vector<EdgeStruct> edges(subgraph->edges_.size());
    for(size_t j = 0; j < subgraph->nodes_.size(); j++)
      nodes[j] = subgraph->nodes_[j];
    for(size_t j = 0; j < subgraph->edges_.size(); j++)
      edges[j] = *((EdgeStruct*) subgraph->edges_[j].data_ptr);
    optimizers_cache_[i] = prepareSolver(nodes, edges);
    cout << "Optimizing subgraph " << i+1 << "/" << subgraphs.size() << endl;
    optimizers_cache_[i]->optimizer_->initializeOptimization();
    optimizers_cache_[i]->optimizer_->optimize(num_iters);
    cout << "done." << endl;
  }
  return subgraphs.size();
}
  
void PoseGraphSlam::getSubgraphs(vector<vector<int> > &subgraph_nodes)
{
  ROS_ASSERT(!update_graph_ && !update_subgraphs_);
  vector<Graph_t::Ptr> subgraphs; getSubgraphs(subgraphs, cached_min_size_);
  subgraph_nodes.resize(subgraphs.size());
  for(size_t i = 0; i < subgraphs.size(); i++)
    subgraph_nodes[i] = subgraphs[i]->nodes_;
}

Eigen::Affine3d PoseGraphSlam::transform(int idx, int *root_idx)
{
  //Fail if called before solve() was called
  ROS_ASSERT(!update_graph_ && !update_subgraphs_);
  if(!has_subgraph_[idx])
  {
    if(root_idx) *root_idx = -1;
    return Eigen::Affine3d(Eigen::Matrix4d::Identity());
  }
  size_t graph_idx = idx_to_subgraph_idx_[idx];
  const VertexSE3* v = dynamic_cast<const VertexSE3*>(
      optimizers_cache_[graph_idx]->optimizer_->vertex(idx));
  ROS_ASSERT(v);
  SE3Quat se3 = v->estimate();
  if(root_idx)
    *root_idx = subgraph_cache_[graph_idx]->nodes_[0];
  return Eigen::Affine3d(se3.to_homogenious_matrix());  // awesome.
}

void PoseGraphSlam::vertexData(int idx, Vector3d* translation, Quaterniond* quat, 
    int *root_idx)
{
  if(!has_subgraph_[idx])
  {
    if(root_idx) *root_idx = -1;
    return;
  }
  int graph_idx = idx_to_subgraph_idx_[idx];
  double data[7];
  optimizers_cache_[graph_idx]->optimizer_->vertex(idx)->getEstimateData(data);
  *translation << data[0], data[1], data[2];
  *quat = Quaterniond(data[6], data[3], data[4], data[5]);
  if(root_idx)
    *root_idx = subgraph_cache_[graph_idx]->nodes_[0];
}

// TODO Alter so it respects the subgraph way of doing things?
size_t PoseGraphSlam::numEdges(int idx)
{
  vector<int> neighbors; getGraph()->getNeighbors(idx, neighbors);
  return neighbors.size();
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
    initialize(n_vert);
    ROS_ASSERT(numNodes() == n_vert);
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
  
PoseGraphSlam::G2OPtr 
PoseGraphSlam::prepareSolver(const vector<int> &nodes, const vector<EdgeStruct> &edges)
{
  //Create the optimizer
  G2OPtr solver(new G2OStruct);
  //Add all nodes
  for(size_t i = 0; i < nodes.size(); i++)
  {
    VertexSE3* v = new VertexSE3;
    v->setId(nodes[i]);
    if(i == 0)
    {
      v->setToOrigin();
      v->setFixed(true);
    }
    solver->optimizer_->addVertex(v);
  }
  //Add all edges
  for(size_t i = 0; i < edges.size(); i++)
  {
    const EdgeStruct &edge = edges[i];
    EdgeSE3* edgeptr = new EdgeSE3;
    edgeptr->vertices()[0] = solver->optimizer_->vertex(edge.idx0);
    edgeptr->vertices()[1] = solver->optimizer_->vertex(edge.idx1);
    Matrix3d rotation = edge.transform.matrix().block(0, 0, 3, 3);
    Vector3d translation = edge.transform.translation();
    SE3Quat se3(rotation, translation);
    edgeptr->setMeasurement(se3);
    edgeptr->setInverseMeasurement(se3.inverse());
    edgeptr->setInformation(edge.covariance.inverse());
    solver->optimizer_->addEdge(edgeptr);
  }
  return solver;
}

PoseGraphSlam::G2OStruct::G2OStruct()
{
  linear_solver_ = new SlamLinearCholmodSolver();
  linear_solver_->setBlockOrdering(false);
  optimizer_ = new g2o::SparseOptimizer;
  solver_ = new SlamBlockSolver(optimizer_, linear_solver_);
  optimizer_->setSolver(solver_);
}

PoseGraphSlam::G2OStruct::~G2OStruct()
{
  //delete linear_solver_;
  //delete solver_;
  delete optimizer_;
}

Graph_t::Ptr PoseGraphSlam::getGraph()
{
  if(update_graph_)
  {
    cout << "Updating graph" << endl;
    graph_cache_ = Graph_t::Ptr(new Graph_t);
    for(size_t i = 0; i < nodes_.size(); i++)
      graph_cache_->addNode(nodes_[i]);
    for(size_t i = 0; i < edges_.size(); i++)
      graph_cache_->addEdge(edges_[i].idx0, edges_[i].idx1, (void*) &edges_[i]);
    update_graph_ = false;
    update_subgraphs_ = true; //Must update subgraphs after doing this
  }
  return graph_cache_;
}

void PoseGraphSlam::getSubgraphs(vector<Graph_t::Ptr> &subgraphs, size_t min_size)
{
  if(update_subgraphs_ || min_size != cached_min_size_)
  {
    cout << "Updating subgraph" << endl;
    Graph_t::Ptr graph = getGraph();
    subgraph_cache_.clear();
    graph->getSubgraphs(subgraph_cache_, min_size);
    update_subgraphs_ = false;
    cached_min_size_ = min_size;
    has_subgraph_.clear();
    idx_to_subgraph_idx_.clear();
    // Update index map
    for(size_t i = 0; i < subgraph_cache_.size(); i++)
      for(size_t j = 0; j < subgraph_cache_[i]->nodes_.size(); j++)
      {
        has_subgraph_[subgraph_cache_[i]->nodes_[j]] = true;
        idx_to_subgraph_idx_[subgraph_cache_[i]->nodes_[j]] = i;
      }
  }
  subgraphs = subgraph_cache_;
}
