#include <xpl_calibration/velo_to_asus_calibrator.h>


using namespace std;
using namespace Eigen;
using namespace rgbd;

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
    int idx = seek(frames_, offset + pcds_[i]->header.stamp * 1e-9 , dt_thresh_);
    if(idx == -1)
      continue;

    pcl::transformPointCloud(*pcds_[i], transformed, transform);
    incrementLoss(frames_[idx], transformed, &val, &count);
  }

  if(count == 0) {
    //ROS_WARN("Number of corresponding pcds is zero.  No objective function terms.  Initial time offset is way off?");
    return std::numeric_limits<double>::max();
  }
  else
    return val / count;
}

void MeanDepthError::incrementLoss(Frame frame, const rgbd::Cloud& pcd, double* val, double* count) const
{
  ROS_ASSERT(frame.depth_->rows() == model_.height_);
  ROS_ASSERT(frame.depth_->cols() == model_.width_);

  // -- Make the ground truth depth image.
  Frame gt;
  model_.cloudToFrame(pcd, &gt);

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
      model_.project(ppt, &gtpt);
      ppt.z_ = frame.depth_->coeffRef(ppt.v_, ppt.u_);
      model_.project(ppt, &pt);

      *val += (pt.getVector3fMap() - gtpt.getVector3fMap()).norm();
      ++(*count);
    }
  }
}

VeloToAsusCalibration::VeloToAsusCalibration() :
  offset_(0)
{
  setVeloToAsus(Affine3f::Identity());
}

void VeloToAsusCalibration::serialize(std::ostream& out) const
{
  out.write((const char*)&offset_, sizeof(double));
  eigen_extensions::serialize(velo_to_asus_.matrix(), out);
}

void VeloToAsusCalibration::deserialize(std::istream& in)
{
  in.read((char*)&offset_, sizeof(double));
  Matrix4f mat;
  eigen_extensions::deserialize(in, &mat);
  setVeloToAsus(Affine3f(mat));  // Also sets asus_to_velo_.
}

std::string VeloToAsusCalibration::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "sync offset: " << offset_ << endl;
  oss << prefix << "transform: " << endl << velo_to_asus_.matrix() << endl;
  return oss.str();
}

VeloToAsusCalibrator::VeloToAsusCalibrator(const rgbd::PrimeSenseModel& model, GridSearchViewHandler* vis) :
  model_(model),
  vis_(vis)
{
}

VeloToAsusCalibration VeloToAsusCalibrator::search() const
{
  cout << "Initializing search using PrimeSenseModel: " << endl;
  cout << model_.status("  ");
  MeanDepthError::Ptr mde(new MeanDepthError(model_, frames_, pcds_));
  GridSearch gs(7);
  gs.verbose_ = false;
  gs.view_handler_ = vis_;
  gs.objective_ = mde;
  gs.num_scalings_ = 7;
  double max_res_time = 0.25;
  double max_res_rot = 5.0 * M_PI / 180.0;
  double max_res_trans = 0.5;
  gs.max_resolutions_ << max_res_time, max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 3;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 4, 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  
  ArrayXd x = gs.search(ArrayXd::Zero(7));
  cout << "GridSearch solution: " << x.transpose() << endl;

  VeloToAsusCalibration cal;
  cal.setVeloToAsus(generateTransform(x(1), x(2), x(3), x(4), x(5), x(6)));
  cal.offset_ = x(0);
  return cal;
}
