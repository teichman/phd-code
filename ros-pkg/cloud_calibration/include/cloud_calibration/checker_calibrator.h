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

namespace cloud_calibration
{
  typedef rgbd::Point Point;
  typedef rgbd::Cloud Cloud;
  using cv::Mat1f;
  using cv::Mat3f;
  using cv::Mat3b;
  using cv::Mat1d;
  using multi_sequence::MultiSequence;

  class CheckerCalibrator
  {
  public:
    CheckerCalibrator();
    CheckerCalibrator(int rows, int cols, float square_size);
    //! For now, does pairwise
    Eigen::Affine3f calibrate( MultiSequence::ConstPtr &seq, 
                              size_t ref_idx, size_t target_idx ) const;
    Eigen::Affine3f calibrateInteractive( MultiSequence::ConstPtr &seq,
                              size_t ref_idx, size_t target_idx ) const;

  protected:
    float focal_length_;
    int checker_rows_, checker_cols_;
    float square_size_;
    //! Populate the camera matrix
    void getCameraMatrix( Mat1d &mat, int img_rows, int img_cols ) const;
    void getDistCoeffs( std::vector<float> &dst ) const;
    //! Populate the objectPoints matrix
    void getObjectPoints( std::vector<cv::Point3f> &pts ) const;
    //! False if a good affine trans was not discoverable
    bool estimateAffineFromFrame( MultiSequence::ConstPtr &seq, 
      size_t frame, size_t ref_idx, size_t target_idx, 
      Eigen::Affine3f &trans, std::vector<Point> &points_ref, std::vector<Point> &points_target,
      bool flip, bool interactive=false) const;
    Eigen::Affine3f estimateAffine( const std::vector<Point> &points_ref,
                                    const std::vector<Point> &points_target) const;
  };
  Eigen::Vector3f pointToEigen( const Point &point );
}

#endif
