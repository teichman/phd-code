#ifndef MEAN_DEPTH_ERROR_H
#define MEAN_DEPTH_ERROR_H

#include <pipeline/params.h>
#include <xpl_calibration/object_matching_calibrator.h>  // For generateTransform.
#include <rgbd_sequence/primesense_model.h>

//! Computes the symmetric MDE between two nearby frames when twiddling the transform between the two.
class FrameAlignmentMDE : public ScalarFunction
{
public:
  typedef boost::shared_ptr<FrameAlignmentMDE> Ptr;
  typedef boost::shared_ptr<const FrameAlignmentMDE> ConstPtr;
  
  //! fraction is how much of the depth data to use.
  //! Default params can come from FrameAligner::defaultParams().
  FrameAlignmentMDE(const pl::Params& params,
                    const rgbd::PrimeSenseModel& model0, const rgbd::PrimeSenseModel& model1,
                    rgbd::Frame frame0, rgbd::Frame frame1,
                    const std::vector<cv::Point2d>& correspondences0 = std::vector<cv::Point2d>(),
                    const std::vector<cv::Point2d>& correspondences1 = std::vector<cv::Point2d>());

  //! x = [rx, ry, rz, tx, ty, tz].
  double eval(const Eigen::VectorXd& x) const;

  //! Number of points from the last eval() which had matches.
  //! Only valid in a single-threaded context.
  double* count_;
  //! The unweighted depth error from the last eval() which had matches.
  //! Only valid in a single-threaded context.
  double* depth_error_;
  
protected:
  pl::Params params_;
  rgbd::PrimeSenseModel model0_;
  rgbd::PrimeSenseModel model1_;
  rgbd::Frame frame0_;
  rgbd::Frame frame1_;
  std::vector<cv::Point2d> correspondences0_;
  std::vector<cv::Point2d> correspondences1_;
  rgbd::Cloud pcd0_;
  rgbd::Cloud pcd1_;
  std::vector<size_t> indices_;
  //! Precomputed images -- hsv, canny, etc
  cv::Mat3f img0_hsv_;
  cv::Mat3f img1_hsv_;
  cv::Mat1b edges0_;
  cv::Mat1b edges1_;
  Eigen::MatrixXf color_names_;
  
};

void transformAndDecimate(const rgbd::Cloud& in,
                          const Eigen::Affine3f& transform,
                          const std::vector<size_t>& indices,
                          rgbd::Cloud* out);


//! Computes the asymmetric MDE for a given set of frames assumed to be from the same sensor
//! and reference pcds when twiddling
//! sync offset and transform between pcds and frames.  It is asymmetric because we do not
//! assume the presence of a projection model for the reference PCDs.
class SequenceAlignmentMDE : public ScalarFunction
{
public:
  //! Frames are from the PrimeSense, pcds are from the Velodyne.
  SequenceAlignmentMDE(const rgbd::PrimeSenseModel& model,
                 const std::vector<rgbd::Frame>& frames,
                 const std::vector<rgbd::Cloud::ConstPtr>& pcds);
  //! x = [sync, rx, ry, rz, tx, ty, tz].
  double eval(const Eigen::VectorXd& x) const;
  
protected:
  rgbd::PrimeSenseModel model_;
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  //! The maximum time difference to allow when choosing frame to pcd correspondences.
  double dt_thresh_;
};

class FocalLengthMDE : public ScalarFunction
{
public:
  //! pcds are assumed to be in the coordinate system of their corresponding frames.
  FocalLengthMDE(const rgbd::PrimeSenseModel& model,
                 const std::vector<rgbd::Frame>& frames,
                 const std::vector<rgbd::Cloud::ConstPtr>& pcds,
                 const std::vector<Eigen::Affine3d>& transforms,
                 double fraction);
  double eval(const Eigen::VectorXd& x) const;

protected:
  rgbd::PrimeSenseModel model_;
  std::vector<rgbd::Frame> frames_;
  std::vector<rgbd::Cloud::ConstPtr> pcds_;
  std::vector<Eigen::Affine3d> transforms_;
  std::vector<size_t> indices_;
};

void meanDepthError(const rgbd::PrimeSenseModel& model,
                    const rgbd::Frame &frame, const rgbd::Cloud& pcd, const rgbd::Frame &gt,
                    double* val, double* count,
                    double max_range = std::numeric_limits<double>::max());

void meanDepthAndColorError(const rgbd::PrimeSenseModel& model,
                            const rgbd::Frame &frame, const rgbd::Cloud& pcd, const rgbd::Frame &gt,
                            double* depth_error, double* color_error, double* count,
                            double max_range = std::numeric_limits<double>::max());

void meanDepthMultiplierAndColorError(const rgbd::PrimeSenseModel& model,
                                      const rgbd::Frame &frame, const rgbd::Cloud& pcd, const rgbd::Frame &gt,
                                      double* depth_error, double* hue_error, double* count,
                                      double max_range = std::numeric_limits<double>::max());
void meanDepthMultiplierAndHueError(const rgbd::PrimeSenseModel& model,
                                      const rgbd::Frame &frame, const rgbd::Cloud& pcd,
              const rgbd::Frame &gt, const rgbd::IndexMap &indexmap,
              const cv::Mat3f &hsv_frame, const cv::Mat3f &hsv_pcd, 
              const std::vector<size_t> &cloud_indices, 
                                      double* depth_error, double* color_error, double* count,
                                      double max_range = std::numeric_limits<double>::max());
void meanDepthMultiplierAndEdgeError(const rgbd::PrimeSenseModel& model,
                                      const rgbd::Frame &frame, const rgbd::Cloud& pcd, 
              const rgbd::Frame &gt, const rgbd::IndexMap &indexmap,
              const cv::Mat1b &edge_frame, const cv::Mat1b &edge_pcd, 
              const std::vector<size_t> &cloud_indices, 
                                      double* depth_error, double* color_error, double* count,
                                      double max_range = std::numeric_limits<double>::max());
void meanDepthMultiplierAndCNError(const rgbd::PrimeSenseModel& model,
                                      const rgbd::Frame &frame, const rgbd::Cloud& pcd,
              const rgbd::Frame &gt,
              const Eigen::MatrixXf &color_names_lookup,
                                      double* depth_error, double* cn_error, double* count,
                                      double max_range = std::numeric_limits<double>::max());

void keypointError(const rgbd::PrimeSenseModel& model0, rgbd::Frame frame0, const std::vector<cv::Point2d> correspondences0,
                   const Eigen::Affine3f& f0_to_f1,
                   const rgbd::PrimeSenseModel& model1, rgbd::Frame frame1, const std::vector<cv::Point2d>& correspondences1,
                   double keypoint_hinge,
                   double* keypoint_error, double* keypoint_error_count);

#endif // MEAN_DEPTH_ERROR_H
