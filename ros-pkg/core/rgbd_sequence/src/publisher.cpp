#include <rgbd_sequence/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>


#define DESYNC (getenv("DESYNC"))
using namespace std;

namespace rgbd
{

  Publisher::Publisher(const std::string& device_id,
		     pcl::OpenNIGrabber::Mode mode) :
    device_id_(device_id),
    mode_(mode),
    grabber_(device_id_, mode, mode),
    it_(nh_)
  {
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image_out",1);
    initializeGrabber();
  }
  
  void Publisher::run()
  {
    grabber_.start();
    while(ros::ok()) {
      usleep(1e3);
    }
    grabber_.stop();
  }

  cv::Mat3b Publisher::oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const
  {
    cv::Mat3b img(oni->getHeight(), oni->getWidth());
    uchar data[img.rows * img.cols * 3];
    oni->fillRGB(img.cols, img.rows, data);
    int i = 0;
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x, i+=3) {
	img(y, x)[0] = data[i+2];
	img(y, x)[1] = data[i+1];
	img(y, x)[2] = data[i];
      }
    }
    
    return img;
  }
  
  void Publisher::imageCallback(const boost::shared_ptr<openni_wrapper::Image>& oni_img)
  {
    cv_bridge::CvImage im;
    im.encoding = sensor_msgs::image_encodings::BGR8;
    oniToCV(oni_img).copyTo(im.image);
    im.header.stamp = ros::Time(oni_img->getTimeStamp() / (double)1e6);
    image_pub_.publish(im.toImageMsg());
  
  }

  void Publisher::initializeGrabber()
  {
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> image_cb;
    image_cb = boost::bind(&Publisher::imageCallback, this, _1);
    grabber_.registerCallback(image_cb);
  }
}
