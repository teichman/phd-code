#ifndef STRUCTURAL_SVM_H
#define STRUCTURAL_SVM_H

#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <optimization/nips.h>
#include <optimization/common_functions.h>
#include <eigen_extensions/eigen_extensions.h>
#include <pipeline2/pipeline2.h>
#include <graphcuts/maxflow_inference.h>

namespace graphcuts
{
  class Constraint;
  class ConstraintGenerator;
  
  class StructuralSVM
  {
  public:
    double c_;
    double precision_;
    int num_threads_;
    int debug_level_;
    
    StructuralSVM(double c, double precision, int num_threads, int debug_level);
    Eigen::VectorXd train(const std::vector<PotentialsCache::Ptr>& caches,
			  const std::vector<VecXiPtr>& labels) const;

  protected:
    double updateWeights(const std::vector<Constraint>& constraints,
			 int num_edge_weights,
			 Eigen::VectorXd* weights,
			 double* slacks) const;
    
    friend class ConstraintGenerator;
  };
  
  class Constraint
  {
  public:
    Constraint() :
      loss_(-1)
      {
      }
    
    Eigen::VectorXd dpsi_;
    double loss_;
  };

  class ConstraintGenerator : public pipeline2::ComputeNode
  {
  public:
    Constraint con_;
    double hamming_loss_;
    
    ConstraintGenerator(const Eigen::VectorXd& weights,
			PotentialsCache::ConstPtr cache,
			VecXiConstPtr labels);
			
    void _compute();
    void _flush();
    std::string _getName() const {return "ConstraintGenerator";}

  protected:
    Eigen::VectorXd weights_;
    PotentialsCache::ConstPtr cache_;
    VecXiConstPtr labels_;
  };

  double hammingLoss(const Eigen::VectorXi& label,
		     const Eigen::VectorXi& pred);
  double zeroOneLoss(const Eigen::VectorXi& label,
		     const Eigen::VectorXi& pred);
  
}

#endif // STRUCTURAL_SVM_H