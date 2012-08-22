#ifndef OPENNI_STREAM_RECORDER_H
#define OPENNI_STREAM_RECORDER_H

#include <XnCppWrapper.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_sequence/stream_sequence.h>
#include <Eigen/Eigen>
#include <pcl/io/openni_camera/openni_image_yuv_422.h>
#include <pcl/io/openni_camera/openni_depth_image.h>

namespace rgbd
{

  class OpenNIStreamRecorder
  {
  public:
    //! mode = {"VGA", "QVGA", "QQVGA"}  Just VGA for now though.
    OpenNIStreamRecorder(const std::string& mode = "VGA", bool registered = false);
    void run();

    static DepthMat oniDepthToEigen(const openni_wrapper::DepthImage& oni);
    static DepthMatPtr oniDepthToEigenPtr(const openni_wrapper::DepthImage& oni);
    //! Warning: This depends on having the patched version of OpenNI that comes with PCL.
    static cv::Mat3b oniToCV(const openni_wrapper::Image& oni);
    
  protected:
    std::string mode_;
    bool recording_;
    StreamSequence::Ptr seq_;
    PrimeSenseModel model_;
    //! Whether registering depth to RGB using OpenNI.
    bool registered_;
    bool visualize_;
    
    xn::Context context_;
    xn::DepthGenerator dgen_;
    xn::ImageGenerator igen_;
    
    void initializeOpenNI();
    void toggleRecording();
  };

}

#endif // OPENNI_STREAM_RECORDER_H


