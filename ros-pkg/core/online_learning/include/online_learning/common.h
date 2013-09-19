#ifndef ONLINE_LEARNING_COMMON_H
#define ONLINE_LEARNING_COMMON_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef boost::shared_ptr<boost::thread> ThreadPtr;
typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Cloud;


#endif // COMMON_H
