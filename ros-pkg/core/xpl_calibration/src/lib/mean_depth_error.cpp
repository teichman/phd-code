#include <xpl_calibration/mean_depth_error.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;


FrameAlignmentMDE::FrameAlignmentMDE(const rgbd::PrimeSenseModel& model0, rgbd::Frame frame0, 
				     const rgbd::PrimeSenseModel& model1, rgbd::Frame frame1,
				     double max_range, double fraction) :
  max_range_(max_range),
  count_(NULL),
  model0_(model0),
  model1_(model1),
  frame0_(frame0),
  frame1_(frame1)
{
  //ScopedTimer st("FrameAlignmentMDE::FrameAlignmentMDE.  frameToClouds.");
  model0_.frameToCloud(frame0_, &pcd0_, max_range_);
  model1_.frameToCloud(frame1_, &pcd1_, max_range_);
  // model0_.frameToCloud(frame0_, &pcd0_);
  // model1_.frameToCloud(frame1_, &pcd1_);
  ROS_ASSERT(pcd0_.size() == pcd1_.size());
  
  // Set up which random pixels to look at.
  // (Calling rand from multiple execution threads is a disaster)
  indices_.reserve(pcd0_.size());
  for(size_t i = 0; i < pcd0_.size(); ++i)
    if((double)rand() / RAND_MAX <= fraction)
      indices_.push_back(i);
}

double FrameAlignmentMDE::eval(const Eigen::VectorXd& x) const
{
  //ScopedTimer st("FrameAlignmentMDE::eval");
  Eigen::Affine3f f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));

  double count = 0;  // Total number of points with both ground truth and measurements.
  double val = 0;  // Total objective.
  Cloud transformed;

  transformAndDecimate(pcd1_, f0_to_f1.inverse(), &transformed);
  meanDepthError(model0_, frame0_, transformed, &val, &count, max_range_);
  transformAndDecimate(pcd0_, f0_to_f1, &transformed);
  meanDepthError(model1_, frame1_, transformed, &val, &count, max_range_);

  // Make count available to other users in single-threaded mode.
  if(count_)
    *count_ = count;
  
  if(count == 0) {
    ROS_WARN("FrameAlignmentMDE found no overlapping points.");
    return std::numeric_limits<double>::max();
  }
  else
    return val / count;
}

void FrameAlignmentMDE::transformAndDecimate(const rgbd::Cloud& in,
					     const Eigen::Affine3f& transform,
					     rgbd::Cloud* out) const
{
  //ScopedTimer st("transformAndDecimate");
  out->clear();
  out->reserve(indices_.size());
  for(size_t i = 0; i < indices_.size(); ++i) {
    size_t idx = indices_[i];
    out->push_back(rgbd::Point());
    out->back().getVector4fMap() = transform * in[idx].getVector4fMap();
    out->back().r = in[idx].r;
    out->back().g = in[idx].g;
    out->back().b = in[idx].b;
  }
}

SequenceAlignmentMDE::SequenceAlignmentMDE(const PrimeSenseModel& model,
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

double SequenceAlignmentMDE::eval(const Eigen::VectorXd& x) const
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
		    double* val, double* count, double max_range)
{
  //ScopedTimer st("meanDepthError total");
  ROS_ASSERT(frame.depth_->rows() == model.height_);
  ROS_ASSERT(frame.depth_->cols() == model.width_);
  HighResTimer hrt;
  
  // -- Make the ground truth depth image.
  hrt.reset("meanDepthError: cloudToFrame"); hrt.start();
  Frame gt;
  model.cloudToFrame(pcd, &gt);
  //hrt.stop(); cout << hrt.report() << endl;

  // -- Count up mean depth error.
  hrt.reset("meanDepthError: counting"); hrt.start();
  ProjectivePoint ppt;
  rgbd::Point pt;
  rgbd::Point gtpt;
  for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
    for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
      // Both ground truth and measurement must have data.
      if(gt.depth_->coeffRef(ppt.v_, ppt.u_) == 0 ||
	 frame.depth_->coeffRef(ppt.v_, ppt.u_) == 0)
      {
      	continue;
      }

      // Ignore measured points beyond max_range.
      if(frame.depth_->coeffRef(ppt.v_, ppt.u_) > max_range * 1000)
      	continue;

      // Ignore points for which both are far away.
      // if(frame.depth_->coeffRef(ppt.v_, ppt.u_) > max_range * 1000 &&
      // 	 gt.depth_->coeffRef(ppt.v_, ppt.u_) > max_range * 1000)
      // {
      // 	continue;
      // }

      ppt.z_ = gt.depth_->coeffRef(ppt.v_, ppt.u_);
      model.project(ppt, &gtpt);
      ppt.z_ = frame.depth_->coeffRef(ppt.v_, ppt.u_);
      model.project(ppt, &pt);

      *val += (pt.getVector3fMap() - gtpt.getVector3fMap()).norm();
      ++(*count);
    }
  }
  //hrt.stop(); cout << hrt.report() << endl;
}
