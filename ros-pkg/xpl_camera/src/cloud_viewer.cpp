/*
 * =====================================================================================
 *
 *       Filename:  cloud_viewer.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  01/19/2012 05:56:11 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stephen Miller (stephen), sdmiller@eecs.berkeley.edu
 *        Company:  UC Berkeley
 *
 * =====================================================================================
 */

#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> KCloud;

class ROSCloudViewer
{
    public:
        pcl::visualization::CloudViewer cloud_viewer_;
        ros::Subscriber cloud_sub_;

        void cloud_callback(const KCloud::ConstPtr& cloud)
        {
            cloud_viewer_.showCloud(cloud);
        }

        ROSCloudViewer(ros::NodeHandle &nh):
            cloud_viewer_("PointCloud")
        {
            cloud_sub_ = nh.subscribe<KCloud>("cloud_in", 2, &ROSCloudViewer::cloud_callback, this);   
        }
        
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_viewer");

  ros::NodeHandle nh;
  ROSCloudViewer cv(nh);
  ros::spin();

  return 0;
}
