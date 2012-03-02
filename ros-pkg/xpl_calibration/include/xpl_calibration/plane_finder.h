#ifndef PLANE_FINDER_H
#define PLANE_FINDER_H

#include <queue>
#include <bag_of_tricks/high_res_timer.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <bag_of_tricks/image_region_iterator.h>

class PlaneFinder
{
public:
  enum {
    PROCESSING = -1,
    UNTOUCHED = -2,
    NONE = -3,
    NODATA = -4
  };
    
  //! 0, 1, 2, ... are assignments of pixel i to that plane.
  //! Negative numbers are special codes.  See the enum.
  std::vector<int> assignments_;
  //! Average surface normal for each plane.
  std::vector<Eigen::Vector3f> normals_;
  //! Number of inliers in each plane.
  //std::vector<int> num_inliers_;
  //! TODO
  std::vector<Eigen::Vector3f> colors_;
  std::vector<cv::Point2i> img_centroids_;
  
  PlaneFinder(size_t min_inliers,
	      double angle_thresh,
	      double distance_thresh);
  void compute(const RGBDCloud& cloud,
	       const pcl::PointCloud<pcl::Normal>& normals);

protected:
  size_t min_inliers_;
  double angle_thresh_;
  double distance_thresh_;

  bool findPlanarSurface(const RGBDCloud& pcd,
			 const pcl::PointCloud<pcl::Normal>& normals,
			 size_t center_idx,
			 int new_plane_id);
  
  void getInPlaneNeighbors(const RGBDCloud& pcd,
			   const pcl::PointCloud<pcl::Normal>& normals,
			   size_t center_idx,
			   const Eigen::Vector3f& plane_normal,
			   double plane_constant,
			   std::vector<int>* indices);
};

class ColorWheel
{
public:
  ColorWheel(int num_colors);
  cv::Vec3b getColor(size_t num) const;
  
private:
  std::vector<cv::Vec3b> colors_;
};

#endif // PLANE_FINDER_H
