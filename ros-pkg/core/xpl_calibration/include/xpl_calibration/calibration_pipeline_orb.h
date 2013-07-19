#ifndef CALIBRATION_PIPELINE_H
#define CALIBRATION_PIPELINE_H

#include <pipeline/pipeline.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <xpl_calibration/frame_selector.h>
#include <xpl_calibration/orb_extractor.h>
#include <xpl_calibration/orb_matcher.h>
#include <xpl_calibration/kdtree_pod.h>
#include <xpl_calibration/transform_validator.h>
#include <xpl_calibration/background_modeler.h>
#include <xpl_calibration/gaussian_background_modeler.h>
#include <xpl_calibration/background_subtractor.h>
#include <xpl_calibration/object_extractor.h>

class CalibrationPipelineOrb
{
public:
  //! Alphanumeric typenames are needed for Pipeline.
  typedef rgbd::Sequence::ConstPtr SequenceConstPtr; 

  //! If no file is given, it will create its own with initializePipeline().
  CalibrationPipelineOrb(int num_threads, std::string pipeline_file = "");
  
  //! Computes transform that will move target to reference.
  //! T * seq1 = seq0.
  Eigen::Affine3f calibrate(rgbd::StreamSequenceBase::ConstPtr seq0,
                            rgbd::StreamSequenceBase::ConstPtr seq1);
  
protected:
  pl::Pipeline pl_;
  void registerPods() const;
  void initializePipeline();
};

#endif // CALIBRATION_PIPELINE_H
