#include <graphcuts/potentials_cache.h>

using namespace Eigen;

namespace graphcuts
{

  long int PotentialsCache::bytes() const
  {
    long int bytes = 0;
    for(size_t i = 0; i < edge_.size(); ++i)
      bytes += sizeof(double) * edge_[i].nonZeros();
    for(size_t i = 0; i < source_.size(); ++i) { 
      bytes += sizeof(double) * source_[i].rows() * source_[i].cols();
      bytes += sizeof(double) * sink_[i].rows() * sink_[i].cols();
    }
    
    return bytes;
  }

  int PotentialsCache::getNumNodes() const
  {
    ROS_ASSERT(sink_.size() == source_.size());
    ROS_ASSERT(!sink_.empty());
    ROS_ASSERT(sink_[0].rows() == source_[0].rows());
    for(size_t i = 1; i < sink_.size(); ++i) {
      ROS_ASSERT(sink_[i].rows() == sink_[i-1].rows());
      ROS_ASSERT(source_[i].rows() == source_[i-1].rows());
    }

    return sink_[0].rows();
  }
  
  int PotentialsCache::getNumPotentials() const
  {
    return getNumNodePotentials() + getNumEdgePotentials();
  }

  int PotentialsCache::getNumNodePotentials() const
  {
    ROS_ASSERT(sink_.size() == source_.size());
    return sink_.size();
  }

  void PotentialsCache::symmetrizeEdges()
  {
    // See http://eigen.tuxfamily.org/api/TutorialSparse.html
    for(size_t i = 0; i < edge_.size(); ++i)
      edge_[i] = 0.5 * (SparseMat(edge_[i].transpose()) + edge_[i]);
  }
  
  void PotentialsCache::applyWeights(const Model& model,
				     SparseMat* edge,
				     Eigen::VectorXd* src,
				     Eigen::VectorXd* snk) const
				     
  {
    ROS_ASSERT(src->rows() == snk->rows());
    ROS_ASSERT(src->rows() == edge->rows());
    ROS_ASSERT(src->rows() == edge->cols());
    ROS_ASSERT(model.npot_weights_.rows() == getNumNodePotentials());
    ROS_ASSERT(model.epot_weights_.rows() == getNumEdgePotentials());
    ROS_ASSERT(src->rows() == edge->cols());
    ROS_ASSERT(!edge_.empty());
    ROS_ASSERT(!source_.empty());
    ROS_ASSERT(source_.size() == sink_.size());

    src->setZero();
    snk->setZero();
    edge->setZero();
    for(size_t i = 0; i < source_.size(); ++i) { 
      *src += model.npot_weights_(i) * source_[i];
      *snk += model.npot_weights_(i) * sink_[i];
    }

    for(size_t i = 0; i < edge_.size(); ++i)
      *edge += model.epot_weights_(i) * edge_[i];
  }

  
  Eigen::VectorXd PotentialsCache::psi(const Eigen::VectorXi& seg) const
  {
    VectorXd psi = VectorXd::Zero(getNumPotentials()); // edge, then node.
    
    // -- Add up scores for edge potentials.
    for(size_t i = 0; i < edge_.size(); ++i) {
      const SparseMat& ep = edge_[i];
      for(int j = 0; j < ep.outerSize(); ++j) {
    	for(SparseMat::InnerIterator it(ep, j); it; ++it) {
    	  if(it.col() <= it.row())
    	    continue;
	  if(seg(it.row()) == seg(it.col()))
	    psi(i) += it.value();
	}
      }
    }

    // -- Add up scores for node potentials.
    for(size_t i = 0; i < source_.size(); ++i) {
      int idx = i + edge_.size();
      ROS_ASSERT(source_[i].size() == seg.rows());
      for(int j = 0; j < seg.rows(); ++j) {
	if(seg(j) == 0)
	  psi(idx) += sink_[i](j);
	else if(seg(j) == 1)
	  psi(idx) += source_[i](j);
	else
	  abort();
      }
    }
     
    return psi;
  }
  
}