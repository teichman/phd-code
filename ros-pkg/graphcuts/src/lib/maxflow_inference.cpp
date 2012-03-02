#include <graphcuts/maxflow_inference.h>

using namespace Eigen;
using namespace std;

namespace graphcuts
{

  MaxflowInference::MaxflowInference(const VectorXd& weights) :
    weights_(weights)
  {
    // for(int i = 0; i < weights_.rows(); ++i)
    //   ROS_ASSERT(weights_(i) > 0);
  }
  
  void MaxflowInference::segment(PotentialsCache::ConstPtr pc,
				 Eigen::VectorXi* seg) const
  {
    // -- Check for monkey business and allocate.
    ROS_ASSERT(pc->getNumPotentials() == weights_.rows());
    ROS_ASSERT(pc->source_.size() == pc->sink_.size());

    if(seg->rows() == 0)
      *seg = VecXi::Zero(pc->source_[0].rows());
      
    for(size_t i = 0; i < pc->source_.size(); ++i) { 
      ROS_ASSERT(seg->rows() == pc->source_[i].rows());
      ROS_ASSERT(seg->rows() == pc->sink_[i].rows());
    }

    int num_edges = 0;
    for(size_t i = 0; i < pc->edge_.size(); ++i) { 
      ROS_ASSERT(seg->rows() == pc->edge_[i].rows());
      ROS_ASSERT(seg->rows() == pc->edge_[i].cols());
      num_edges += pc->edge_[i].nonZeros();
    }
    
    Graph3d graph(seg->rows(), num_edges);
    graph.add_node(seg->rows());
		        
    Eigen::VectorXd weighted_source(seg->rows());
    Eigen::VectorXd weighted_sink(seg->rows());
    SparseMat weighted_edge(seg->rows(), seg->rows());
    pc->applyWeights(weights_, &weighted_source, &weighted_sink, &weighted_edge);
    
    // -- Fill the graph with weighted node potentials.
    for(int i = 0; i < weighted_source.rows(); ++i)
     graph.add_tweights(i, weighted_source(i), weighted_sink(i));
    
    // -- Fill the graph with edge potentials.  Assumes symmetry & upper triangular.
    for(int i = 0; i < weighted_edge.outerSize(); ++i) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(weighted_edge, i); it; ++it) {
	if(it.col() <= it.row())
	  continue;

	ROS_WARN_STREAM_COND(it.value() < 0, "Edgepot weighted sum is negative: " << it.value());
	ROS_FATAL_STREAM_COND(isnan(it.value()), "NaN in edgepot.");
	graph.add_edge(it.row(), it.col(), it.value(), it.value());
      }
    }

    // -- Run graph cuts.
    HighResTimer hrt("maxflow");
    hrt.start();
    graph.maxflow();
    hrt.stop();
    //cout << hrt.reportMilliseconds() << endl;

    // -- Fill the segmentation.
    for(int i = 0; i < seg->rows(); ++i) {
      if(graph.what_segment(i, Graph3d::SINK) == Graph3d::SINK)
	seg->coeffRef(i) = 0;
      else
	seg->coeffRef(i) = 1;
    }
  }
  
}
