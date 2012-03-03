#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_sequence/rgbd_sequence.h>

namespace rgbd
{

  class Recorder
  {
  public:
    //! 640x480: OpenNI_VGA_30Hz
    //! 160x120: OpenNI_QQVGA_30Hz
    Recorder(const std::string& device_id = "",
	     pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz);
    void cloudCallback(const Cloud::ConstPtr& cloud);
    void imageCallback(const boost::shared_ptr<openni_wrapper::Image>& image);
    void irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img);
    void depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni);
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    void run();

  protected:
    std::string device_id_;
    pcl::OpenNIGrabber::Mode mode_;
    pcl::OpenNIGrabber grabber_;
    pcl::visualization::CloudViewer cloud_viewer_;
    bool recording_;
    std::vector<Cloud::ConstPtr> clouds_;
    std::vector<cv::Mat3b> imgs_;
    std::vector<double> image_timestamps_;

    cv::Mat3b oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const;
    cv::Mat1b irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const;
    void initializeGrabber();
    void toggleRecording();
  };

}
