#include <xpl_calibration/mocap_detector.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

using namespace std;

MocapDetector::MocapDetector(int checker_cols, int checker_rows, float square_size):
  checker_cols_(checker_cols), checker_rows_(checker_rows), square_size_(square_size)
{
  tl_ = Eigen::Vector3f(-square_size_-0.0175,-square_size_-0.050,-0.025);
  tr_ = Eigen::Vector3f(6*square_size_+0.025,-square_size_-0.050,-0.025);
  bl_ = Eigen::Vector3f(0.055, 8*square_size_+0.0475, -0.025);
  br_ = Eigen::Vector3f(4*square_size_+0.05, 8*square_size_+0.0525, -0.025);
}

bool MocapDetector::locatePoints(const StreamSequence::ConstPtr &seq, size_t frame, 
    Point &tl, Point &tr, Point& bl, Point &br) const
{
    Cloud::Ptr pcd = seq->getCloud(frame);
    cv::Mat3b img = seq->getImage(frame);
    // Get checkerboard
    vector<cv::Point2f> corners;
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK;
    if(!cv::findChessboardCorners(img, cv::Size(checker_cols_, checker_rows_), corners, flags) )
      return false;
    Cloud::Ptr checker_pcd( new Cloud );
    pcl::PointIndices::Ptr checker_indices( new pcl::PointIndices);
    for(size_t j = 0; j < corners.size(); j++){
      const cv::Point2f& pt = corners[j];
      checker_pcd->points.push_back(pcd->at(pt.x,pt.y));
      //cv::Mat3b drawn_img;
      //img.copyTo(drawn_img);
      //cout << "On pt " << j << endl;
      //cv::circle(drawn_img, pt, 5, cv::Scalar(127,127,127), -1);
      //cv::imshow("Image", drawn_img);
      //cv::waitKey();
    }
    pcl::SACSegmentation<Point> plane_fitter;
    pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    plane_fitter.setModelType(pcl::SACMODEL_PLANE);
    plane_fitter.setMethodType(pcl::SAC_RANSAC);
    plane_fitter.setDistanceThreshold(0.01);
    plane_fitter.setInputCloud(checker_pcd);
    plane_fitter.segment(*inliers, *coeffs);
    //Project board points to plane
    Cloud::Ptr checker_pts_projected(new Cloud);
    float a = coeffs->values[0];
    float b = coeffs->values[1];
    float c = coeffs->values[2];
    float d = coeffs->values[3];
    for(size_t j = 0; j < corners.size(); j++){
      const cv::Point2f board_pt = corners[j];
      float x = (board_pt.x - seq->cx_) / seq->fx_;
      float y = (board_pt.y - seq->cy_) / seq->fy_;
      float t = -d / (a*x + b*y + c);
      Point pt_proj;
      pt_proj.x = x*t;
      pt_proj.y = y*t;
      pt_proj.z = t;
      pt_proj.b = pt_proj.g = pt_proj.r = 127;
      checker_pts_projected->points.push_back(pt_proj);
    }
    //Compute board_x, board_y;
    Eigen::Vector3f dx, dy, dz, origin;
    origin = checker_pts_projected->points[0].getVector3fMap();
    dx = (checker_pts_projected->points[1].getVector3fMap() - origin) / square_size_;
    dy = (checker_pts_projected->points[checker_cols_].getVector3fMap() - origin) / square_size_;
    dz = Eigen::Vector3f(a,b,c);
    Eigen::Matrix3f proj;
    proj.col(0) = dx; proj.col(1) = dy; proj.col(2) = dz;
    Eigen::Vector3f tl_vec, tr_vec, bl_vec, br_vec;
    tl_vec = origin + proj*tl_;
    tr_vec = origin + proj*tr_;
    bl_vec = origin + proj*bl_;
    br_vec = origin + proj*br_;
    tl.x = tl_vec(0); tl.y = tl_vec(1); tl.z = tl_vec(2); tl.r = 255; tl.g = 0; tl.b = 0;
    tr.x = tr_vec(0); tr.y = tr_vec(1); tr.z = tr_vec(2); tr.r = 255; tr.g = 0; tr.b = 0;
    bl.x = bl_vec(0); bl.y = bl_vec(1); bl.z = bl_vec(2); bl.r = 255; bl.g = 0; bl.b = 0;
    br.x = br_vec(0); br.y = br_vec(1); br.z = br_vec(2); br.r = 255; br.g = 0; br.b = 0;
    return true;
}
