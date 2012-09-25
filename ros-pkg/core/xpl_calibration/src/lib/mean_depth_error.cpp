#include <xpl_calibration/mean_depth_error.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

//#define TIMING



// Takes points from frame0, turns them in to lines in the coordinate system of frame1, then finds how far keypoints in frame1 are
// from the lines they should lie on.
void keypointError(const rgbd::PrimeSenseModel& model0, rgbd::Frame frame0, const std::vector<cv::Point2d> correspondences0,
		   const Eigen::Affine3f& f0_to_f1,
		   const rgbd::PrimeSenseModel& model1, rgbd::Frame frame1, const std::vector<cv::Point2d>& correspondences1,
		   double* keypoint_error, double* keypoint_error_count)
{
  ROS_ASSERT(correspondences0.size() == correspondences1.size());
  
  // -- Find the location of the origin of frame0 in the image of frame1.
  Point originpt;
  originpt.getVector3fMap() = f0_to_f1.translation();
  ProjectivePoint originppt;
  model1.project(originpt, &originppt);
  
  ProjectivePoint ppt;
  Point pt;
  for(size_t i = 0; i < correspondences0.size(); ++i) {
    
    // -- Get a 3D test point along the ray.
    ppt.u_ = correspondences0[i].x;
    ppt.v_ = correspondences0[i].y;
    ppt.z_ = 5000;

    Point testpt;
    model0.project(ppt, &testpt);
    testpt.getVector3fMap() = f0_to_f1 * testpt.getVector3fMap();
    
    // -- Project the test point and origin point into frame1.
    ProjectivePoint testppt;
    model1.project(testpt, &testppt);

    // Ignore edge case.  Could do distance to point here, but that probably doesn't really matter.
    if(originppt.u_ == testppt.u_ && originppt.v_ == testppt.v_)
      continue;
    
    // -- Get the error for this keypoint.
    Vector2d origin;
    origin(0) = originppt.u_;
    origin(1) = originppt.v_;
    Vector2d test;
    test(0) = testppt.u_;
    test(1) = testppt.v_;
    Vector2d v = test - origin;
    Vector2d normal;
    normal(0) = -v(1);
    normal(1) = v(0);
    normal.normalize();
    double b = normal.dot(origin);
    if(!(fabs(b - normal.dot(test)) < 1e-6)) { 
      ROS_FATAL_STREAM("b: " << b << ", origin: " << origin.transpose() << ", v: " << v.transpose() << ", normal: " << normal.transpose() << ", test: " << test.transpose() << ", normal.dot(test): " << normal.dot(test));
      abort();
    }
    Vector2d p;
    p(0) = correspondences1[i].x;
    p(1) = correspondences1[i].y;

    Vector2d pp = p - (normal.dot(p) - b) / (normal.dot(normal)) * normal;
    //    cout << "fabs(normal.dot(pp) - b): " << fabs(normal.dot(pp) - b) << endl;
    ROS_ASSERT(fabs(normal.dot(pp) - b) < 1e-6);
    *keypoint_error += min(50.0, fabs(normal.dot(p - pp)));
    //*keypoint_error += fabs(normal.dot(p - pp));
    ++(*keypoint_error_count);
  }
}

FrameAlignmentMDE::FrameAlignmentMDE(const rgbd::PrimeSenseModel& model0, const rgbd::PrimeSenseModel& model1,
				     rgbd::Frame frame0, rgbd::Frame frame1,
				     const std::vector<cv::Point2d>& correspondences0, const std::vector<cv::Point2d>& correspondences1,
				     double max_range, double fraction) :
  max_range_(max_range),
  count_(NULL),
  model0_(model0),
  model1_(model1),
  frame0_(frame0),
  frame1_(frame1),
  correspondences0_(correspondences0),
  correspondences1_(correspondences1)
{
#ifdef TIMING
  ScopedTimer st("FrameAlignmentMDE::FrameAlignmentMDE.  frameToClouds.");
#endif

  ROS_ASSERT(!model0_.hasDepthDistortionModel());
  ROS_ASSERT(!model1_.hasDepthDistortionModel());
  ROS_ASSERT(correspondences0_.size() == correspondences1_.size());

  model0_.frameToCloud(frame0_, &pcd0_, max_range_);
  model1_.frameToCloud(frame1_, &pcd1_, max_range_);
  ROS_ASSERT(pcd0_.size() == pcd1_.size());

  
  // Set up which random pixels to look at.
  // (Calling rand from multiple execution threads is a disaster)
  indices_.reserve(pcd0_.size());
  for(size_t i = 0; i < pcd0_.size(); ++i)
    if((double)rand() / RAND_MAX <= fraction)
      indices_.push_back(i);

  cv::Mat3b vis0 = frame0.img_.clone();
  cv::Mat3b vis1 = frame1.img_.clone();
  for(size_t i = 0; i < correspondences0.size(); ++i) {
    cv::Scalar color(rand() % 255, rand() % 255, rand() % 255);
    cv::circle(vis0, correspondences0[i], 2, color, -1);
    cv::circle(vis1, correspondences1[i], 2, color, -1);
  }
  cv::imshow("vis0", vis0);
  cv::imshow("vis1", vis1);
  cv::waitKey(5);
}

double FrameAlignmentMDE::eval(const Eigen::VectorXd& x) const
{
#ifdef TIMING
  ScopedTimer st("FrameAlignmentMDE::eval");
#endif 
  Eigen::Affine3f f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));

  double count = 0;  // Total number of points with both ground truth and measurements.
  double val = 0;  // Total objective.
  double depth_error = 0;
  double color_error = 0;
  double keypoint_error = 0;
  double keypoint_error_count = 0;
  Cloud transformed;

  transformAndDecimate(pcd1_, f0_to_f1.inverse(), indices_, &transformed);
  meanDepthAndColorError(model0_, frame0_, transformed, &depth_error, &color_error, &count, max_range_);
  keypointError(model0_, frame0_, correspondences0_, f0_to_f1, model1_, frame1_, correspondences1_, &keypoint_error, &keypoint_error_count);
  
  transformAndDecimate(pcd0_, f0_to_f1, indices_, &transformed); 
  meanDepthAndColorError(model1_, frame1_, transformed, &depth_error, &color_error, &count, max_range_);
  keypointError(model1_, frame1_, correspondences1_, f0_to_f1.inverse(), model0_, frame0_, correspondences0_, &keypoint_error, &keypoint_error_count);

  // Make count available to other users in single-threaded mode.
  if(count_)
    *count_ = count;

  int min_correspondences = 20;
  if(keypoint_error_count < min_correspondences) {
    return numeric_limits<double>::max();
  }
  else
    keypoint_error /= keypoint_error_count;

  double min_points = 100;
  if(count < min_points) {
    ROS_WARN("FrameAlignmentMDE had < min_points overlapping 3d points.");
    return numeric_limits<double>::max();
  }
  else {
    depth_error /= count;
    color_error /= count;
  }

  //cout << "Num correspondences used for keypoint error: " << keypoint_error_count << ", Keypoint error: " << keypoint_error << endl;
  //val = 0.01 * keypoint_error;
  //val = depth_error + 0.0023 * color_error;

  // Color error term has a per-pixel max of 441.
  // Keypoint error is hinged at 50.  It's probably a bit weaker than the 3D data, though, so we want it
  // to just nudge things when the 3D doesn't really have a preference.
  double depth_term = depth_error;
  double color_term = 0.000115 * color_error;
  double keypoint_term = 0.005 * keypoint_error;
  val = depth_term + color_term + keypoint_term;
  //cout << "Depth: " << depth_term << ", color: " << color_term << ", keypoint: " << keypoint_term << ", total: " << val << endl;

  return val;
}

void transformAndDecimate(const rgbd::Cloud& in,
			  const Eigen::Affine3f& transform,
			  const std::vector<size_t>& indices,
			  rgbd::Cloud* out)
{
#ifdef TIMING
  ScopedTimer st("transformAndDecimate");
#endif
  out->clear();
  out->reserve(indices.size());  
  for(size_t i = 0; i < indices.size(); ++i) {
    size_t idx = indices[i];
    if(idx >= in.size())
      break;
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
  dt_thresh_(0.005)
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
  int num_pcds = 0;
  for(size_t i = 0; i < pcds_.size(); ++i) {
    int idx = seek(frames_, offset + pcds_[i]->header.stamp * 1e-9, dt_thresh_);
    if(idx == -1)
      continue;

    pcl::transformPointCloud(*pcds_[i], transformed, transform);
    meanDepthError(model_, frames_[idx], transformed, &val, &count);
    ++num_pcds;
  }

  double min_count = 100 * num_pcds;
  if(count < min_count) {
    ROS_WARN_STREAM("Number of corresponding pcds is less than the threshold of " << min_count);
    return std::numeric_limits<double>::max();
  }
  else
    return val / count;
}

FocalLengthMDE::FocalLengthMDE(const PrimeSenseModel& model,
			       const std::vector<Frame>& frames,
			       const std::vector<Cloud::ConstPtr>& pcds,
			       const std::vector<Eigen::Affine3d>& transforms,
			       double fraction) :
  model_(model),
  frames_(frames),
  pcds_(pcds),
  transforms_(transforms)
{
  // Set up which random pixels to look at.
  // (Calling rand from multiple execution threads is a disaster)
  size_t max_num_pts = 0;
  for(size_t i = 0; i < pcds.size(); ++i)
    max_num_pts = max(max_num_pts, pcds[i]->size());
  
  indices_.reserve(max_num_pts);
  for(size_t i = 0; i < max_num_pts; ++i)
    if((double)rand() / RAND_MAX <= fraction)
      indices_.push_back(i);
}

double FocalLengthMDE::eval(const Eigen::VectorXd& x) const
{
  PrimeSenseModel model = model_;
  model.fx_ = x(0);
  model.fy_ = x(0);

  double count = 0;  // Total number of points with both ground truth and measurements.
  double val = 0;  // Total objective.
  Cloud transformed;
  for(size_t i = 0; i < pcds_.size(); ++i) {
    transformAndDecimate(*pcds_[i], transforms_[i].cast<float>(), indices_, &transformed);
    //pcl::transformPointCloud(*pcds_[i], transformed, transforms_[i].cast<float>());
    meanDepthError(model, frames_[i], transformed, &val, &count);
  }

  double min_count = 100 * pcds_.size();
  if(count < min_count) {
    ROS_WARN_STREAM("Number of corresponding pcds is less than the threshold of " << min_count);
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
#ifdef TIMING
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 

  // -- Count up mean depth error.
  hrt.reset("meanDepthError: counting"); hrt.start();
  ProjectivePoint ppt;
  rgbd::Point pt;
  rgbd::Point gtpt;
  double max_range_mm = max_range * 1000;
  double val_mm = 0;
  for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
    for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
      // -- Both ground truth and measurement must have data.
      double gtz = gt.depth_->coeffRef(ppt.v_, ppt.u_);
      if(gtz == 0)
	continue;
      double z = frame.depth_->coeffRef(ppt.v_, ppt.u_);
      if(z == 0)
	continue;
      
      // -- Ignore measured points beyond max_range.
      if(z > max_range_mm)
      	continue;

      // -- Ignore points for which both are far away.
      // if(frame.depth_->coeffRef(ppt.v_, ppt.u_) > max_range * 1000 &&
      // 	 gt.depth_->coeffRef(ppt.v_, ppt.u_) > max_range * 1000)
      // {
      // 	continue;
      // }


      // -- Count up range error.
      // ppt.z_ = gt.depth_->coeffRef(ppt.v_, ppt.u_);
      // model.project(ppt, &gtpt);
      // ppt.z_ = frame.depth_->coeffRef(ppt.v_, ppt.u_);
      // model.project(ppt, &pt);
      // *val += (pt.getVector3fMap() - gtpt.getVector3fMap()).norm();

      // -- Count up z error.
      val_mm += fabs(z - gtz);
      
      ++(*count);
    }
  }
  *val += val_mm * 0.001;
  
#ifdef TIMING
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 
}

void meanDepthAndColorError(const rgbd::PrimeSenseModel& model,
			    Frame frame, const rgbd::Cloud& pcd,
			    double* depth_error, double* color_error,
			    double* count, double max_range)
{
  ROS_ASSERT(frame.depth_->rows() == model.height_);
  ROS_ASSERT(frame.depth_->cols() == model.width_);
  HighResTimer hrt;
  
  // -- Make the ground truth depth image.
  hrt.reset("meanDepthError: cloudToFrame"); hrt.start();
  Frame gt;
  model.cloudToFrame(pcd, &gt);
#ifdef TIMING
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 

  // -- Count up mean depth error.
  hrt.reset("meanDepthError: counting"); hrt.start();
  ProjectivePoint ppt;
  rgbd::Point pt;
  rgbd::Point gtpt;
  double max_range_mm = max_range * 1000;
  double depth_error_mm = 0;
  double local_color_error = 0;

  for(ppt.u_ = 0; ppt.u_ < gt.depth_->cols(); ++ppt.u_) {
    for(ppt.v_ = 0; ppt.v_ < gt.depth_->rows(); ++ppt.v_) {
      // -- Both ground truth and measurement must have data.
      double gtz = gt.depth_->coeffRef(ppt.v_, ppt.u_);
      if(gtz == 0)
	continue;
      double z = frame.depth_->coeffRef(ppt.v_, ppt.u_);
      if(z == 0)
	continue;
      
      // -- Ignore measured points beyond max_range.
      if(z > max_range_mm)
      	continue;

      // -- Count up z error.
      depth_error_mm += fabs(z - gtz);

      // -- Count up color error.
      cv::Vec3b gtc = gt.img_(ppt.v_, ppt.u_);
      cv::Vec3b c = frame.img_(ppt.v_, ppt.u_);
      local_color_error += sqrt((gtc[0] - c[0]) * (gtc[0] - c[0]) +
      				(gtc[1] - c[1]) * (gtc[1] - c[1]) +
      				(gtc[2] - c[2]) * (gtc[2] - c[2]));
      //local_color_error += fabs((double)gtc[0] - c[0]) + fabs((double)gtc[1] - c[1]) + fabs((double)gtc[2] - c[2]);
	      
      ++(*count);
    }
  }
  *depth_error += depth_error_mm * 0.001;
  *color_error += local_color_error;
  
#ifdef TIMING
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
#endif 
}
