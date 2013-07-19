#ifndef XPL_CALIBRATOR_H
#define XPL_CALIBRATOR_H

#include <xpl_calibration/common.h>

class TransformValidator : public pl::Pod
{
public:
  typedef std::vector<Eigen::Affine3f> Candidates;
  typedef pcl::KdTreeFLANN<rgbd::Point> KdTree;
  
  DECLARE_POD(TransformValidator);
  TransformValidator(std::string name) :
    Pod(name)
  {
    declareParam<double>("Gamma", 0.1); // Weighting of the color difference term.
    declareParam<int>("MaxTuningIters", 10);
    declareParam<int>("Skip", 10); // Use every kth point.

    declareInput<const Candidates*>("Candidates");
    declareInput<rgbd::Cloud::ConstPtr>("Cloud0");
    declareInput<rgbd::Cloud::ConstPtr>("Cloud1");
    declareInput<KdTree::Ptr>("KdTree0");
    declareInput<KdTree::Ptr>("KdTree1");

    declareOutput<const Eigen::Affine3f*>("BestTransform");
  }

  void compute();
  void debug() const;
  
protected:
  //! 0 to 1
  std::vector<Eigen::Affine3f> candidates_;
  double best_loss_;
  Eigen::Affine3f best_transform_;
  
  double computeLoss(const rgbd::Cloud& cloud0,
                     KdTree& tree0,
                     const rgbd::Cloud& cloud1) const;
  
  void fineTuneAlignment(const rgbd::Cloud& cloud0,
                         KdTree& tree0,
                         const rgbd::Cloud& cloud1,
                         Eigen::Affine3f* transform) const;

};

#endif // XPL_CALIBRATOR_H
