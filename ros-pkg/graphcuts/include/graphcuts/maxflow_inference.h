#ifndef GRAPHCUTS_MAXFLOW_INFERENCE_H
#define GRAPHCUTS_MAXFLOW_INFERENCE_H

#include <bag_of_tricks/high_res_timer.h>
#include <graphcuts/potentials_cache.h>

namespace graphcuts
{

  class MaxflowInference
  {
  public:
    //! Edge potential weights, then node potential weights.
    Eigen::VectorXd weights_;
    
    MaxflowInference(const Eigen::VectorXd& weights);
    //! seg must be the right size.
    void segment(PotentialsCache::ConstPtr potentials, Eigen::VectorXi* seg) const;
    
  };

  
  
}

#endif // GRAPHCUTS_MAXFLOW_INFERENCE_H
