#ifndef STRUCT_SVM_H
#define STRUCT_SVM_H

#include <qpOASES/QProblem.hpp>
#include <eigen_extensions/eigen_extensions.h>
#include <dst/segmentation_pipeline.h>

namespace dst
{

  class Constraint
  {
  public:
    Eigen::VectorXd dpsi_;
    double loss_;
    int tr_ex_id_;
  };
  
  class StructSVM
  {
  public:
    StructSVM(double c, double precision, bool margin_rescaling);
    void loadSequences(const std::string& path);
    //! Assumes that previous segmentation up until this point is perfect.
    Eigen::VectorXd train();
    //! Iteratively A) runs DS&T, adding constraints for each frame until segmentation
    //! gets really bad, then B) learns new weights.
    Eigen::VectorXd trainRobust();
    Eigen::VectorXd trainRobust2();
    
  protected:
    double c_;
    double precision_;
    bool margin_rescaling_;
    std::vector<KinectSequence::Ptr> sequences_;

    int getNumWeights() const { SegmentationPipeline sp(1); return sp.getWeights().rows(); } 
    Eigen::VectorXd runSolver(const std::vector<FramePotentialsCache::Ptr>& caches,
			      const std::vector<cv::Mat1b>& labels) const;
    void addConstraint(const SegmentationPipeline& sp, int id,
		       cv::Mat1b label, cv::Mat1b ymv, double loss,
		       std::vector<Constraint>* constraints) const;
      
    double loss(cv::Mat1b label, cv::Mat1b pred, bool hamming) const;
    static double* eigToQPO(const Eigen::VectorXd& eig);
    static double* eigToQPO(const Eigen::MatrixXd& eig);
    double updateWeights(const std::vector<Constraint>& constraints,
			 int num_tr_ex,
			 int num_edge_weights,
			 Eigen::VectorXd* weights,
			 Eigen::VectorXd* slacks,
			 bool* feas) const;
    double qpOASES(const Eigen::MatrixXd& H,
		   const Eigen::VectorXd& g,
		   const Eigen::MatrixXd& A,
		   const Eigen::VectorXd& lb,
		   const Eigen::VectorXd& lbA,
		   Eigen::VectorXd* x,
		   bool* feas) const;
	
  };

} 
  
#endif // STRUCT_SVM_H
