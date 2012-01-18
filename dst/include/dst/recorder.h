#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dst/typedefs.h>
#include <dst/lockable.h>
#include <dst/kinect_sequence.h>

namespace dst
{

  class Recorder : public Lockable
  {
  public:
    Recorder(const std::string& device_id = "");
    void cloudCallback(const KinectCloud::ConstPtr& cloud);
    void imageCallback(const boost::shared_ptr<openni_wrapper::Image>& image);
    void irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img);
    void depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni);
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    void run();

  protected:
    std::string device_id_;
    pcl::OpenNIGrabber grabber_;
    pcl::visualization::CloudViewer cloud_viewer_;
    bool recording_;
    std::vector<KinectCloud::ConstPtr> clouds_;
    std::vector<cv::Mat3b> images_;
    std::vector<double> image_timestamps_;

    cv::Mat3b oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const;
    cv::Mat1b irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const;
    void initializeGrabber();
    void toggleRecording();
  };

}
