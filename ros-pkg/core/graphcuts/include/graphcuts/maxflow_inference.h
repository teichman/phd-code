#ifndef GRAPHCUTS_MAXFLOW_INFERENCE_H
#define GRAPHCUTS_MAXFLOW_INFERENCE_H

#include <timer/timer.h>
#include <graphcuts/potentials_cache.h>
#include <graphcuts/model.h>

namespace gc
{
  
  class MaxflowInference
  {
  public:
    Model model_;
    
    MaxflowInference(const Model& model);
    //! seg is resized if necessary, and contains elements in {-1, 0, +1}^n.
    //! Returns the max flow in the graph, equivalent to the cost of the min cut (i.e. segmentation).
    //! If the max flow is large, it was hard to do the segmentation.
    double segment(PotentialsCache::ConstPtr potentials, Eigen::VectorXi* seg) const;
  };
  
}

#endif // GRAPHCUTS_MAXFLOW_INFERENCE_H
