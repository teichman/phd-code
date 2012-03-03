#ifndef CALIBRATION_PIPELINE_H
#define CALIBRATION_PIPELINE_H

#include <pipeline/pipeline.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <xpl_calibration/frame_selector.h>
#include <xpl_calibration/orb_extractor.h>

class CalibrationPipelineOrb
{
public:
  //! Alphanumeric typenames are needed for Pipeline.
  typedef rgbd::Sequence::ConstPtr SequenceConstPtr; 
  
  CalibrationPipelineOrb(int num_threads);
  //! Computes transform that will move target to reference.
  //! T * target = reference.
  Eigen::Affine3f calibrate(rgbd::Sequence::ConstPtr reference,
			    rgbd::Sequence::ConstPtr target);
  
protected:
  pipeline::Pipeline pl_;
  void registerPods() const;
  void initializePipeline();
};

#endif // CALIBRATION_PIPELINE_H
