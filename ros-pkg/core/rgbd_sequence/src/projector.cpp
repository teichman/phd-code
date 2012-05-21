#include <rgbd_sequence/projector.h>

using namespace std;

namespace rgbd
{

  void ProjectedPoint::serialize(std::ostream& out) const
  {
    out << setprecision(16) << u_ << " " << v_ << " " << z_ << endl;
  }

  void ProjectedPoint::deserialize(std::istream& in)
  {
    in >> u_;
    in >> v_;
    in >> z_;
  }

  Projector::Projector() :
    fx_(-1),
    fy_(-1),
    cx_(-1),
    cy_(-1),
    width_(-1),
    height_(-1)
  {
  }
  
  void Projector::frameToCloud(const Frame& frame, Cloud* pcd) const
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
    for(int v = 0; v < dm.rows(); ++v) {
      for(int u = 0; u < dm.cols(); ++u, ++idx) {
	Point& pt = pcd->at(idx);
	unsigned short z = dm(v, u);
	if(z == 0) {
	  pt.x = std::numeric_limits<float>::quiet_NaN();
	  pt.y = std::numeric_limits<float>::quiet_NaN();
	  pt.z = std::numeric_limits<float>::quiet_NaN();
	  continue;
	}
	pt.z = z * 0.001;
	pt.x = pt.z * (u - cx_) / fx_;
	pt.y = pt.z * (v - cy_) / fy_;
	pt.r = img(v, u)[2];
	pt.g = img(v, u)[1];
	pt.b = img(v, u)[0];
      }
    }
  }
  
  void Projector::cloudToFrame(const Cloud& pcd, Frame* frame) const
  {
    ROS_ASSERT(pcd.isOrganized());
    ROS_ASSERT((int)pcd.width == width_);
    ROS_ASSERT((int)pcd.height == height_);
    
    frame->depth_ = DepthMat(pcd.height, pcd.width);
    frame->img_ = cv::Mat3b(cv::Size(pcd.width, pcd.height));
    ROS_ASSERT(frame->img_.cols == (int)pcd.width);
    ROS_ASSERT(frame->img_.rows == (int)pcd.height);
    frame->timestamp_ = pcd.header.stamp.toSec();

    int idx = 0;
    for(size_t v = 0; v < pcd.height; ++v) {
      for(size_t u = 0; u < pcd.width; ++u, ++idx) {
	const Point& pt = pcd[idx];
	if(!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z)) { 
	  frame->depth_(v, u) = 0;
	  frame->img_(v, u) = cv::Vec3b(0, 0, 0);
	}
	else { 
	  frame->depth_(v, u) = (unsigned short)(pt.z * 1000.0);
	  frame->img_(v, u) = cv::Vec3b(pt.b, pt.g, pt.r);
	}
      }
    }
  }

  void Projector::project(const ProjectedPoint& ppt, Point* pt) const
  {
    ROS_ASSERT(ppt.u_ >= 0 && ppt.v_ >= 0 && ppt.u_ < width_ && ppt.v_ < height_);
    project(fx_, fy_, cx_, cy_, ppt, pt);
  }

  void Projector::project(const Point& pt, ProjectedPoint* ppt) const
  {
    ROS_ASSERT(isFinite(pt));

    ppt->u_ = pt.x * fx_ / pt.z + cx_;
    ppt->v_ = pt.y * fy_ / pt.z + cy_;
    ppt->z_ = pt.z * 1000;
  }

  void Projector::project(double fx, double fy, double cx, double cy,
			  const ProjectedPoint& ppt, Point* pt)
  {
    if(ppt.z_ == 0) {
      pt->x = std::numeric_limits<float>::quiet_NaN();
      pt->y = std::numeric_limits<float>::quiet_NaN();
      pt->z = std::numeric_limits<float>::quiet_NaN();
      return;
    }

    pt->z = ppt.z_ * 0.001;
    pt->x = pt->z * (ppt.u_ - cx) / fx;
    pt->y = pt->z * (ppt.v_ - cy) / fy;
  }

  
} // namespace rgbd
