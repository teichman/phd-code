#include <slam_interface/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#ifdef PCL_ROS
#include <pcl_ros/point_cloud.h>
#endif

#include <ros/ros.h>

using namespace std;
using rgbd::StreamSequence;

namespace slam_interface
{

  Publisher::Publisher() :
    pausetime_(1.0),
    maxdist_(2.55)
  {
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image_out",1);
    depth_pub_ = nh_.advertise<sensor_msgs::Image>("depth_out",1);
    image_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("image_info_out",1);
    depth_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("depth_info_out",1);
#ifdef PCL_ROS
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("points_out", 1);
#endif
    //image_pub_ = it_.advertise("image_out",1);
    //depth_pub_ = it_.advertise("depth_out",1);
  }
  
  void Publisher::run(const StreamSequence::ConstPtr &seq)
  {
    for(size_t i = 0; i < seq->size(); i++)
    {
      cout << "Publishing image " << i+1 << " of " << seq->size() << endl;
      //Publish RGB
      cv_bridge::CvImage im;
      im.encoding = sensor_msgs::image_encodings::RGB8;
      im.header.stamp = i * 1e9;//ros::Time::now();
      cv::cvtColor(seq->getImage(i),im.image, CV_BGR2RGB);
      image_pub_.publish(im.toImageMsg());
      //Publish both camera infos
      sensor_msgs::CameraInfo info;
      info.header.stamp = im.header.stamp;
      info.height = im.image.rows;
      info.width = im.image.cols;
      info.distortion_model = "plumb_bob";
      //Set K
      double fx, fy, cx, cy;
      seq->getIntrinsics(i, fx, fy, cx, cy);
      info.K[0] = fx; info.K[1] = 0;  info.K[2] = cx; 
      info.K[3] = 0;  info.K[4] = fy; info.K[5] = cy; 
      info.K[6] = 0;  info.K[7] = 0;  info.K[8] = 1;
      //Set R
      info.R[0] = 1.; info.R[1] = 0;  info.R[2] = 0; 
      info.R[3] = 0;  info.R[4] = 1.; info.R[5] = 0; 
      info.R[6] = 0;  info.R[7] = 0;  info.R[8] = 1.;
      //Set P
      info.P[0] = fx; info.P[1] = 0;  info.P[2] = cx; info.P[3] = 0;
      info.P[4] = 0;  info.P[5] = fy; info.P[6] = cy; info.P[7] = 0;
      info.P[8] = 0;  info.P[9] = 0;  info.P[10] = 1; info.P[11] = 0;
      //Publish RGB 
      info.header.frame_id = "/openni_rgb_optical_frame";
      image_info_pub_.publish(info);
      //Now do the same for depth
      info.header.frame_id = "/openni_depth_optical_frame";
      depth_info_pub_.publish(info);
      //Publish cloud
      rgbd::Cloud::Ptr cloud = seq->getCloud(i);
      cloud->header.frame_id = "/openni_rgb_optical_frame";
      cloud->header.stamp = im.header.stamp;
      sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloud, *cloud_msg);
#ifdef PCL_ROS
      point_pub_.publish(cloud_msg);
#endif
      //Publish depth
      cv_bridge::CvImage dp;
      dp.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      dp.header.stamp = im.header.stamp;
      cv::Mat1f dp_mat(im.image.rows, im.image.cols);
      assert(cloud->height==im.image.rows && cloud->width==im.image.cols);
      for(size_t ii = 0; ii < dp_mat.rows; ii++)
      {
        for(size_t jj = 0; jj < dp_mat.cols; jj++)
        {
          dp_mat(ii,jj) = 0;
          float value = cloud->operator()(jj,ii).z;
          //Saturate at maxdist_
          if(value > maxdist_)
            value = std::numeric_limits<float>::quiet_NaN();
          dp_mat(ii,jj) = value;
        }
      }
      dp.image = dp_mat;
      depth_pub_.publish(dp.toImageMsg());
      //Sleep
      ros::spinOnce();
      usleep(pausetime_*1E6);
    }
  }

}
