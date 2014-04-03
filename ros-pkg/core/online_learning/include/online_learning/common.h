#ifndef ONLINE_LEARNING_COMMON_H
#define ONLINE_LEARNING_COMMON_H

#include <boost/shared_ptr.hpp>

namespace pcl {
class Normal;
class PointXYZRGB;
template<class T> class PointCloud;
namespace search {
template<class T> class KdTree;
}
}
typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Cloud;
typedef boost::shared_ptr<Cloud> CloudPtr;
typedef boost::shared_ptr<const Cloud> CloudConstPtr;
typedef pcl::PointCloud<pcl::Normal> NormalsCloud;
typedef pcl::search::KdTree<pcl::PointXYZRGB> KdTree;

#endif // COMMON_H
