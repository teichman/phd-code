#ifndef GRAPHCUTS_POTENTIALS_CACHE_H
#define GRAPHCUTS_POTENTIALS_CACHE_H

#include <ros/assert.h>
#include <ros/console.h>
#include <graphcuts/typedefs.h>
#include <graphcuts/model.h>

namespace graphcuts
{

  //! Represents a function that maps segmentations to score vectors
  //! i.e. \Psi_x : seg -> R^n.
  //! w^T \Psi_x(y) is the score for segmentation y on frame with features x.
  class PotentialsCache
  {
  public:
    typedef boost::shared_ptr<PotentialsCache> Ptr;
    typedef boost::shared_ptr<const PotentialsCache> ConstPtr;

    
    //! In order of weights.
    std::vector<SparseMat> edge_;
    //! In order of weights.
    std::vector<Eigen::VectorXd> sink_;
    std::vector<Eigen::VectorXd> source_;
    NameMapping2 epot_names_;
    NameMapping2 npot_names_;
    
    long int bytes() const;
    int getNumNodes() const;
    int getNumPotentials() const;
    int getNumNodePotentials() const;
    int getNumEdgePotentials() const { return edge_.size(); }


    //! weights.dot(psi()) is the score for a given segmentation.
    //! Concatenates edge, then node potentials.
    Eigen::VectorXd psi(const Eigen::VectorXi& seg) const;

    void symmetrizeEdges();
    void applyWeights(const Model& model,
		      SparseMat* edge,
		      Eigen::VectorXd* src,
		      Eigen::VectorXd* snk) const;
		      
  };

}

#endif // GRAPHCUTS_POTENTIALS_CACHE_H
