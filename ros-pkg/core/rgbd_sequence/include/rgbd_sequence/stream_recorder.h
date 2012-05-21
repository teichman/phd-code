#ifndef STREAM_RECORDER_H
#define STREAM_RECORDER_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_sequence/stream_sequence.h>
#include <Eigen/Eigen>

namespace rgbd
{

  class StreamRecorder
  {
  public:
    //! 640x480: OpenNI_VGA_30Hz
    //! 160x120: OpenNI_QQVGA_30Hz
    StreamRecorder(const std::string& device_id = "",
		   pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz, 
		   const std::string& calib_file="",
		   bool view_cloud = false);
    void cloudCallback(const Cloud::ConstPtr& cloud);
    void irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img);
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
    void rgbdCallback(const boost::shared_ptr<openni_wrapper::Image>& rgb,
		      const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
		      float f_inv);
    void run();

    static DepthMat oniDepthToEigen(const boost::shared_ptr<openni_wrapper::DepthImage>& oni);
    static DepthMatPtr oniDepthToEigenPtr(const boost::shared_ptr<openni_wrapper::DepthImage>& oni);
    static cv::Mat3b oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni);
    
  protected:
    std::string device_id_;
    pcl::OpenNIGrabber::Mode mode_;
    pcl::OpenNIGrabber grabber_;
    pcl::visualization::CloudViewer cloud_viewer_;
    bool recording_;
    StreamSequence::Ptr seq_;
    bool view_cloud_;
    //Calibration info
    std::string calib_file_;
    float fx_, fy_, cx_, cy_;
    int image_width_, image_height_;
    bool manual_calibration_;
    
    cv::Mat1b irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const;
    void initializeGrabber();
    void toggleRecording();
    void initializeCalibration();
  };

}

#endif // STREAM_RECORDER_H


