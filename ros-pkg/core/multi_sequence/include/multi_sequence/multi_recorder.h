#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_sequence/rgbd_sequence.h>

namespace multi_sequence
{
  using namespace rgbd;

  class MultiRecorder
  {
  public:
    //! 640x480: OpenNI_VGA_30Hz
    //! 160x120: OpenNI_QQVGA_30Hz
    MultiRecorder(const std::vector<std::string> &device_ids,
             pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz);
    void cloudCallback(const Cloud::ConstPtr& cloud, size_t idx);
    void imageCallback(const boost::shared_ptr<openni_wrapper::Image>& image, size_t idx);
    void irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img);
    void depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni);
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    void run();

  protected:
    std::vector<std::string> device_ids_;
    pcl::OpenNIGrabber::Mode mode_;
    std::vector<pcl::OpenNIGrabber*> grabbers_;
    std::vector<pcl::visualization::CloudViewer*> cloud_viewers_;
    bool recording_;
    std::vector<std::vector<Cloud::ConstPtr> > clouds_;
    std::vector<std::vector<cv::Mat3b> > imgs_;
    std::vector<std::vector<double> > image_timestamps_;

    cv::Mat3b oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const;
    cv::Mat1b irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const;
    void initializeGrabber(size_t idx);
    void toggleRecording();
  };

}

