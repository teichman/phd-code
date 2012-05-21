#ifndef __RGBD_SEQUENCE_PUBLISHER_H__
#define __RGBD_SEQUENCE_PUBLISHER_H__

#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>


namespace rgbd
{

  class Publisher
  {
  public:
    //! 640x480: OpenNI_VGA_30Hz
    //! 160x120: OpenNI_QQVGA_30Hz
    Publisher(const std::string& device_id = "",
	     pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz);
    void imageCallback(const boost::shared_ptr<openni_wrapper::Image>& image);
    void run();


  protected:
    std::string device_id_;
    pcl::OpenNIGrabber::Mode mode_;
    pcl::OpenNIGrabber grabber_;
    //ROS things below
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher image_pub_;

    cv::Mat3b oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const;
    void initializeGrabber();
  };

}

#endif
