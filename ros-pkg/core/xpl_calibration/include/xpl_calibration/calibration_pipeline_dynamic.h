#ifndef CALIBRATION_PIPELINE_DYNAMIC_H
#define CALIBRATION_PIPELINE_DYNAMIC_H

#include <pcl/visualization/cloud_viewer.h>
#include <pipeline/pipeline.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <xpl_calibration/frame_selector.h>
#include <xpl_calibration/kdtree_pod.h>
#include <xpl_calibration/transform_validator.h>
#include <xpl_calibration/background_modeler.h>
#include <xpl_calibration/background_subtractor.h>
#include <xpl_calibration/object_extractor.h>
#include <xpl_calibration/object_matching_calibrator.h>
#include <xpl_calibration/common.h>

class CalibrationPipelineDynamic
{
public:
  //! Alphanumeric typenames are needed for Pipeline.
  typedef rgbd::Sequence::ConstPtr SequenceConstPtr; 

  //! If no file is given, it will create its own with initializePipeline().
  CalibrationPipelineDynamic(int num_threads, std::string pipeline_file = "");
  
  //! Computes transform that will move target to reference.
  //! T * seq1 = seq0.
  void calibrate(rgbd::StreamSequenceBase::ConstPtr seq0,
                 rgbd::StreamSequenceBase::ConstPtr seq1,
                 Eigen::Affine3f* transform,
                 double* sync);
      
protected:
  pl::Pipeline pl_;
  void registerPods() const;
  void initializePipeline();
};

#endif // CALIBRATION_PIPELINE_DYNAMIC_H
