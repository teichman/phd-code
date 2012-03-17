#include <rgbd_sequence/projector.h>

namespace rgbd
{

  Projector::Projector() :
    fx_(-1),
    fy_(-1),
    cx_(-1),
    cy_(-1)
  {
  }
  
  void Projector::frameToCloud(const Frame& frame, Cloud* pcd) const
  {
    const DepthMat& dm = frame.depth_;
    cv::Mat3b img = frame.img_;

    ROS_ASSERT(fx_ > 0 && fy_ > 0 && cx_ > 0 && cy_ > 0);
    ROS_ASSERT(dm.rows() == img.rows);
    ROS_ASSERT(dm.cols() == img.cols);
    
    pcd->clear();
    pcd->height = dm.rows();
    pcd->width = dm.cols();
    pcd->is_dense = false;
    pcd->resize(dm.rows() * dm.cols());
    pcd->header.stamp.fromSec(frame.timestamp_);

    Point bad_point;
    bad_point.x = std::numeric_limits<float>::quiet_NaN();
    bad_point.y = std::numeric_limits<float>::quiet_NaN();
    bad_point.z = std::numeric_limits<float>::quiet_NaN();
    int idx = 0;
    for(int v = 0; v < dm.rows(); ++v) {
      for(int u = 0; u < dm.cols(); ++u, ++idx) {
	Point& pt = pcd->at(idx);
	unsigned short z = dm(v, u);
	if(z == 0) {
	  pt = bad_point;
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

} // namespace rgbd
