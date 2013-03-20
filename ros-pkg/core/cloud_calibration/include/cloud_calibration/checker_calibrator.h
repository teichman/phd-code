#ifndef __CLOUD_CALIBRATION_CHECKER_CALIBRATOR_H__
#define __CLOUD_CALIBRATION_CHECKER_CALIBRATOR_H__

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <multi_sequence/multi_sequence.h>
#include <rgbd_sequence/stream_sequence_base.h>

namespace cloud_calibration
{
  typedef rgbd::Point Point;
  typedef rgbd::Cloud Cloud;
  using cv::Mat1f;
  using cv::Mat3f;
  using cv::Mat3b;
  using cv::Mat1d;
  using multi_sequence::MultiSequence;
  using rgbd::StreamSequenceBase;

  class CheckerCalibrator
  {
  public:
    CheckerCalibrator();
    CheckerCalibrator(int rows, int cols);
    //! Calibrate a pair of sequences
    Eigen::Affine3f calibrate( const StreamSequenceBase::Ptr &seq_ref, const StreamSequenceBase::Ptr &seq_target, 
                               float &std_trans, float &std_rot,
                               double dt_thresh=0.05) const;
    //! Calibrate a multi sequence
    Eigen::Affine3f calibrate( const MultiSequence::ConstPtr &seq, 
                              size_t ref_idx, size_t target_idx, 
                              std::vector<Point> &points_ref, std::vector<Point> &points_target,
                              float &std_trans, float &std_rot) const;

    Eigen::Affine3f estimateAffine( const std::vector<Point> &points_ref,
                                    const std::vector<Point> &points_target) const;
  protected:
    int checker_rows_, checker_cols_;
    //! False if a good affine trans was not discoverable
    bool estimateAffineFromFrame( const MultiSequence::ConstPtr &seq, 
      size_t frame, size_t ref_idx, size_t target_idx, 
      Eigen::Affine3f &trans, std::vector<Point> &points_ref, std::vector<Point> &points_target,
      bool flip, bool fast_check=true, bool interactive=false) const;
  };
  Eigen::Vector3f pointToEigen( const Point &point );
}

#endif
