#include <xpl_calibration/mean_depth_error.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;


FrameAlignmentMDE::FrameAlignmentMDE(const rgbd::PrimeSenseModel& model0, rgbd::Frame frame0, 
				     const rgbd::PrimeSenseModel& model1, rgbd::Frame frame1) :
  model0_(model0),
  model1_(model1),
  frame0_(frame0),
  frame1_(frame1)
{
  model0_.frameToCloud(frame0_, &pcd0_);
  model1_.frameToCloud(frame1_, &pcd1_);
}

double FrameAlignmentMDE::eval(const Eigen::VectorXd& x) const
{
  Eigen::Affine3f f1_to_f0 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));

  double count = 0;  // Total number of points with both ground truth and measurements.
  double val = 0;  // Total objective.
  Cloud transformed;

  pcl::transformPointCloud(pcd1_, transformed, f1_to_f0);
  meanDepthError(model0_, frame0_, transformed, &val, &count);
  pcl::transformPointCloud(pcd0_, transformed, f1_to_f0.inverse());
  meanDepthError(model1_, frame1_, transformed, &val, &count);

  if(count == 0) {
    ROS_WARN("FrameAlignmentMDE found no overlapping points.");
    return std::numeric_limits<double>::max();
  }
  else
    return val / count;
}

MeanDepthError::MeanDepthError(const PrimeSenseModel& model,
			       const std::vector<Frame>& frames,
			       const std::vector<Cloud::ConstPtr>& pcds) :
  model_(model),
  frames_(frames),
  pcds_(pcds),
  dt_thresh_(0.015)
{
}

int seek(const std::vector<Frame>& frames, double ts1, double dt_thresh)
{
  int idx = -1;
  double min = numeric_limits<double>::max();
  // TODO: This could be faster than linear search.
  for(size_t i = 0; i < frames.size(); ++i) {
    double ts0 = frames[i].timestamp_;
    double dt = fabs(ts0 - ts1);
    if(dt < min) {
      min = dt;
      idx = i;
    }
  }

  if(min < dt_thresh)
    return idx;
  else
    return -1;
}

double MeanDepthError::eval(const Eigen::VectorXd& x) const
{
  double offset = x(0);
  Eigen::Affine3f transform = generateTransform(x(1), x(2), x(3), x(4), x(5), x(6));

  double count = 0;  // Total number of points with both ground truth and measurements.
  double val = 0;  // Total objective.
  Cloud transformed;
  for(size_t i = 0; i < pcds_.size(); ++i) {
    int idx = seek(frames_, offset + pcds_[i]->header.stamp.toSec(), dt_thresh_);
    if(idx == -1)
      continue;

    pcl::transformPointCloud(*pcds_[i], transformed, transform);
    meanDepthError(model_, frames_[idx], transformed, &val, &count);
  }

  if(count == 0) {
    //ROS_WARN("Number of corresponding pcds is zero.  No objective function terms.  Initial time offset is way off?");
    return std::numeric_limits<double>::max();
  }
  else
    return val / count;
}

void meanDepthError(const rgbd::PrimeSenseModel& model,
		    Frame frame, const rgbd::Cloud& pcd,
		    double* val, double* count)
{
  ROS_ASSERT(frame.depth_->rows() == model.height_);
  ROS_ASSERT(frame.depth_->cols() == model.width_);

  // -- Make the ground truth depth image.
  Frame gt;
  model.cloudToFrame(pcd, &gt);

  // -- Count up mean depth error.
  ProjectivePoint ppt;
  rgbd::Point pt;
  rgbd::Point gtpt;
  for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
    for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
      // Both ground truth and measurement must have data.
      if(gt.depth_->coeffRef(ppt.v_, ppt.u_) == 0 || frame.depth_->coeffRef(ppt.v_, ppt.u_) == 0)
      	continue;
      
      ppt.z_ = gt.depth_->coeffRef(ppt.v_, ppt.u_);
      model.project(ppt, &gtpt);
      ppt.z_ = frame.depth_->coeffRef(ppt.v_, ppt.u_);
      model.project(ppt, &pt);

      *val += (pt.getVector3fMap() - gtpt.getVector3fMap()).norm();
      ++(*count);
    }
  }
}
