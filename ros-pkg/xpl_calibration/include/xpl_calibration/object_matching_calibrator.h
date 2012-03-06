#ifndef OBJECT_MATCHING_CALIBRATOR_H
#define OBJECT_MATCHING_CALIBRATOR_H

#include <xpl_calibration/common.h>

class ObjectMatchingCalibrator : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > Objects;
  
  DECLARE_POD(ObjectMatchingCalibrator);
  ObjectMatchingCalibrator(std::string name) :
    Pod(name)
  {
    declareInput<const Objects*>("Objects0");
    declareInput<const Objects*>("Objects1");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    declareOutput<const Eigen::Affine3f*>("RoughTransform"); // Based on centroid matching only.
  }

  void compute();
  void debug() const;

protected:
  Eigen::Affine3f rough_transform_;
};

#endif // OBJECT_MATCHING_CALIBRATOR_H
