#ifndef ASP_H
#define ASP_H

//#include <pipeline/pipeline.h>
#include <graphcuts/potentials_cache.h>
#include <name_mapping/name_mapping.h>

// This is bad.  Don't ever do this.
namespace gc = graphcuts;
//namespace pl = pipeline;

class AbstractSegmentationPipeline
{
public:
  //pl::Pipeline pipeline_;

  //! Initializes pipeline_ with NodePotentialAggregator, EdgePotentialAggregator, 
  //! and GraphCuts pods.  You then hook up your own things to this.
  AbstractSegmentationPipeline();

  void setWeights(const ASPWeights& weights);
  //! Names are automatically filled in based on Pipeline pod names.
  ASPWeights getWeights() const;
  //! Runs structural SVM solver.  Does not set weights.
  ASPWeights train(const std::vector<gc::PotentialsCache::ConstPtr>& caches,
		   const std::vector<gc::VecXiConstPtr>& labels) const;
  //! You have to set the input to the pipeline yourself.
  //! segmentation will be resized if it is not the correct length.
  void segment(Eigen::VectorXi* segmentation);
  //! Potentials from the last run of segment().  You generally only run
  //! this when building a training set.
  void getPotentialsCache(gc::PotentialsCache* pc) const;
  
};

#endif // ASP_H
