#ifndef ONI_RECORDER_H
#define ONI_RECORDER_H

#include <XnCppWrapper.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_sequence/stream_sequence.h>
#include <Eigen/Eigen>
#include <pcl/io/openni_camera/openni_image_yuv_422.h>
#include <pcl/io/openni_camera/openni_image_bayer_grbg.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <rgbd_sequence/synchronizer.h>
#include <gperftools/profiler.h>

namespace rgbd
{

  class OniRecorder
  {
  public:
    //! mode \in {"VGA", "QVGA", "QQVGA"}  Just VGA for now though.
    //! type \in {"xpl", "kinect"}
    //! id is the device number.
    OniRecorder(const std::string& type,
		int id,
		const std::string& mode = "VGA",
		bool registered = false);
    void run();

  protected:
    std::string mode_;
    bool recording_;
    PrimeSenseModel model_;
    //! Whether registering depth to RGB using OpenNI.
    bool registered_;
    bool frame_sync_;
    bool visualize_;

    double prev_depth_ts_;
    
    xn::Context context_;
    xn::DepthGenerator dgen_;
    xn::ImageGenerator igen_;
    double max_cycle_time_;
    double total_cycle_time_;
    double total_cycles_;

    void initializeOpenNI();
    bool toggleRecording();
    void getRGBD();
    cv::Vec3b colorize(double depth, double min_range, double max_range) const;
    void handleXnStatus(const XnStatus& status) const;
  };

}

#endif // ONI_RECORDER_H


