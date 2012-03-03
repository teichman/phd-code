#ifndef CALIBRATION_PIPELINE_H
#define CALIBRATION_PIPELINE_H

#include <pipeline/pipeline.h>

class CalibrationPipelineOrb
{
public:
  //! Alphanumeric typenames are needed for Pipeline.
  typedef Sequence::ConstPtr SequenceConstPtr; 
  
  CalibrationPipelineOrb(int num_threads);
  //! Computes transform that will move target to reference.
  //! T * target = reference.
  Eigen::Affine3f calibrate(rgbd::Sequence::ConstPtr reference,
			    rgbd::Sequence::ConstPtr target);
  
protected:
  Pipeline pl_;
  void registerPods() const;
  void initializePipeline();
};

#endif // CALIBRATION_PIPELINE_H
