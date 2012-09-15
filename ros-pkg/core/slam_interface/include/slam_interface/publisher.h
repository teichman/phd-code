#ifndef __SLAM_INTERFACE_PUBLISHER_H__
#define __SLAM_INTERFACE_PUBLISHER_H__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

#include <rgbd_sequence/stream_sequence.h>

namespace slam_interface
{

  class Publisher
  {
  public:
    //! 640x480: OpenNI_VGA_30Hz
    //! 160x120: OpenNI_QQVGA_30Hz
    Publisher();
    void run(const rgbd::StreamSequence::ConstPtr &seq);

    //! Pause duration in seconds
    float pausetime_;
    //! Distance after which readings will be ignored
    float maxdist_;
    //! Amount to subsample frames by. N where we want to view 1/N of the frames.
    int subsample_;
  protected:
    //ROS things below
    ros::NodeHandle nh_;
    //image_transport::ImageTransport it_;
    ros::Publisher image_pub_, depth_pub_;
#ifdef PCL_ROS
    ros::Publisher point_pub_;
#endif
    ros::Publisher image_info_pub_, depth_info_pub_;
  };

}

#endif //__SLAM_INTERFACE_PUBLISHER_H__
