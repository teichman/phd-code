#ifndef GRAPHCUTS_MAXFLOW_INFERENCE_H
#define GRAPHCUTS_MAXFLOW_INFERENCE_H

#include <bag_of_tricks/high_res_timer.h>
#include <graphcuts/potentials_cache.h>
#include <graphcuts/model.h>

namespace graphcuts
{
  
  class MaxflowInference
  {
  public:
    Model model_;
    
    MaxflowInference(const Model& model);
    //! seg must be the right size.
    void segment(PotentialsCache::ConstPtr potentials, Eigen::VectorXi* seg) const;
  };
  
}

#endif // GRAPHCUTS_MAXFLOW_INFERENCE_H
