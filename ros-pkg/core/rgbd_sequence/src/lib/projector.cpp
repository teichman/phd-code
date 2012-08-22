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
    getline(in, &buf);
  }

  PrimeSenseModel::PrimeSenseModel() :
    fx_(-1),
    fy_(-1),
    cx_(-1),
    cy_(-1),
    width_(-1),
    height_(-1)
  {
  }
  
  void PrimeSenseModel::frameToCloud(const Frame& frame, Cloud* pcd) const
  {    
    const DepthMat& dm = frame.depth_;
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
    for(int ppt.v_ = 0; ppt.v_ < dm.rows(); ++ppt.v_) {
      for(int ppt.u_ = 0; ppt.u_ < dm.cols(); ++ppt.u_, ++idx) {
	ppt.z_ = dm(ppt.v_, ppt.u_);
	ppt.r_ = img(v, u)[2];
	ppt.g_ = img(v, u)[1];
	ppt.b_ = img(v, u)[0];
	
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
  
} // namespace rgbd
