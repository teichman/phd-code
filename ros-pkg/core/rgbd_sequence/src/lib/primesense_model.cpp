#include <rgbd_sequence/primesense_model.h>

using namespace std;

namespace rgbd
{

  void ProjectivePoint::serialize(std::ostream& out) const
  {
    out << setprecision(16) << u_ << " " << v_ << " " << z_ << " " << (int)r_ << " " << (int)g_ << " " << (int)b_ << endl;
  }

  void ProjectivePoint::deserialize(std::istream& in)
  {
    in >> u_;
    in >> v_;
    in >> z_;
    in >> r_;
    in >> g_;
    in >> b_;

    string buf;
    getline(in, buf);
  }

  PrimeSenseModel::PrimeSenseModel() :
    type_("unknown"),
    id_(-1),
    width_(-1),
    height_(-1),
    fx_(-1),
    fy_(-1),
    cx_(-1),
    cy_(-1)
  {
  }
  
  void PrimeSenseModel::frameToCloud(const Frame& frame, Cloud* pcd) const
  {    
    const DepthMat& dm = *frame.depth_;
    cv::Mat3b img = frame.img_;

    ROS_ASSERT(fx_ > 0 && fy_ > 0 && cx_ > 0 && cy_ > 0);
    ROS_ASSERT(dm.rows() == img.rows);
    ROS_ASSERT(dm.cols() == img.cols);
    ROS_ASSERT(img.rows == height_);
    ROS_ASSERT(img.cols == width_);
    
    pcd->clear();
    pcd->height = dm.rows();
    pcd->width = dm.cols();
    pcd->is_dense = false;
    pcd->resize(dm.rows() * dm.cols());
    pcd->header.stamp.fromSec(frame.timestamp_);

    int idx = 0;
    ProjectivePoint ppt;
    for(ppt.v_ = 0; ppt.v_ < dm.rows(); ++ppt.v_) {
      for(ppt.u_ = 0; ppt.u_ < dm.cols(); ++ppt.u_, ++idx) {
	ppt.z_ = dm(ppt.v_, ppt.u_);
	ppt.r_ = img(ppt.v_, ppt.u_)[2];
	ppt.g_ = img(ppt.v_, ppt.u_)[1];
	ppt.b_ = img(ppt.v_, ppt.u_)[0];
	
	Point& pt = pcd->at(idx);
	project(ppt, &pt);
      }
    }
  }
  
  void PrimeSenseModel::project(const ProjectivePoint& ppt, Point* pt) const
  {
    ROS_ASSERT(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_);

    pt->r = ppt.r_;
    pt->g = ppt.g_;
    pt->b = ppt.b_;

    if(ppt.z_ == 0) {
      pt->x = std::numeric_limits<float>::quiet_NaN();
      pt->y = std::numeric_limits<float>::quiet_NaN();
      pt->z = std::numeric_limits<float>::quiet_NaN();
    }
    else {
      pt->z = ppt.z_ * 0.001;
      pt->x = pt->z * (ppt.u_ - cx_) / fx_;
      pt->y = pt->z * (ppt.v_ - cy_) / fy_;
    }
  }

  void PrimeSenseModel::project(const Point& pt, ProjectivePoint* ppt) const
  {
    ROS_ASSERT(isFinite(pt));

    ppt->u_ = pt.x * fx_ / pt.z + cx_;
    ppt->v_ = pt.y * fy_ / pt.z + cy_;
    ppt->z_ = pt.z * 1000;
    ppt->r_ = pt.r;
    ppt->g_ = pt.g;
    ppt->b_ = pt.b;
  }
  
  bool PrimeSenseModel::initialized() const
  {
    if(cx_ == -1 || cy_ == -1)
      return false;
    if(fx_ == -1 || fy_ == -1)
      return false;
    if(width_ == -1 || height_ == -1)
      return false;

    return true;
  }

  void PrimeSenseModel::serialize(std::ostream& out) const
  {
    ROS_ASSERT(type_ == "xpl" || type_ == "kinect");
    ROS_ASSERT(id_ != -1);
    ROS_ASSERT(width_ != -1);
    ROS_ASSERT(height_ != -1);
    ROS_ASSERT(fx_ != -1);
    ROS_ASSERT(fy_ != -1);
    ROS_ASSERT(cx_ != -1);
    ROS_ASSERT(cy_ != -1);

    out << type_ << endl;
    eigen_extensions::serializeScalar(id_, out);
    eigen_extensions::serializeScalar(width_, out);
    eigen_extensions::serializeScalar(height_, out);
    eigen_extensions::serializeScalar(fx_, out);
    eigen_extensions::serializeScalar(fy_, out);
    eigen_extensions::serializeScalar(cx_, out);
    eigen_extensions::serializeScalar(cy_, out);
  }

  void PrimeSenseModel::deserialize(std::istream& in)
  {
    getline(in, type_);
    ROS_ASSERT(type_ == "xpl" || type_ == "kinect");
    eigen_extensions::deserializeScalar(in, &id_);
    eigen_extensions::deserializeScalar(in, &width_);
    eigen_extensions::deserializeScalar(in, &height_);
    eigen_extensions::deserializeScalar(in, &fx_);
    eigen_extensions::deserializeScalar(in, &fy_);
    eigen_extensions::deserializeScalar(in, &cx_);
    eigen_extensions::deserializeScalar(in, &cy_);
  }

  std::string PrimeSenseModel::status(const std::string& prefix) const
  {
    ostringstream oss;
    oss << prefix << "type: " << type_ << endl;
    oss << prefix << "id: " << id_ << endl;
    oss << prefix << "size: " << width_ << " x " << height_ << endl;
    oss << prefix << "fx: " << fx_ << endl;
    oss << prefix << "fy: " << fy_ << endl;
    oss << prefix << "cx: " << cx_ << endl;
    oss << prefix << "cy: " << cy_ << endl;
    return oss.str();
  }
  
} // namespace rgbd

