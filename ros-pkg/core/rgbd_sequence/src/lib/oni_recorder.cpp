#include <rgbd_sequence/oni_recorder.h>
#include <bag_of_tricks/high_res_timer.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  OniRecorder::OniRecorder(const std::string& type,
			   int id,
			   const std::string& mode,
			   bool registered) :
    mode_(mode),
    recording_(true),
    registered_(registered),
    visualize_(true),
    max_cycle_time_(0),
    total_cycle_time_(0),
    total_cycles_(0)
  {
    model_.type_ = type;
    model_.id_ = id;
    initializeOpenNI();
  }
  
  void OniRecorder::run()
  {
    while(true) {
      char key = cv::waitKey(2);
      bool done = false;
      if(key == 'q')
	break;
      if(key == ' ')
	done = toggleRecording();

      if(done)
	break;
      
      if(recording_)
	getRGBD();
    }
  }

  void OniRecorder::handleXnStatus(const XnStatus& status) const
  {
    if(status != XN_STATUS_OK) {
      ROS_FATAL_STREAM("OpenNI failure:" << endl << xnGetStatusString(status));
      ROS_ASSERT(status == XN_STATUS_OK);
    }
  }
      
  void OniRecorder::getRGBD()
  {
    HighResTimer hrt("cycle time");
    hrt.start();
    XnStatus retval = context_.WaitOneUpdateAll(dgen_);
    if(retval != XN_STATUS_OK) {
      printf("Failed updating data: %s\n", xnGetStatusString(retval));
      return;
    }
    hrt.stop();

    double ct = hrt.getMilliseconds();
    max_cycle_time_ = max(max_cycle_time_, ct);
    total_cycle_time_ += ct;
    ++total_cycles_;
    cout << "Current cycle time: " << fixed << setw(7) << setprecision(4) << ct;
    cout << ", mean cycle time: " << fixed << setw(7) << setprecision(4) << total_cycle_time_ / total_cycles_;
    cout << ", max cycle time: " << fixed << setw(7) << setprecision(4) << max_cycle_time_;
    cout << endl;
  }

  cv::Vec3b OniRecorder::colorize(double depth, double min_range, double max_range) const
  {
    if(depth == 0)
      return cv::Vec3b(0, 0, 0);
    
    double increment = (max_range - min_range) / 3;
    double thresh0 = min_range;
    double thresh1 = thresh0 + increment;
    double thresh2 = thresh1 + increment;
    double thresh3 = thresh2 + increment;
    
    if(depth < thresh0) {
      return cv::Vec3b(0, 0, 255);
    }
    if(depth >= thresh0 && depth < thresh1) {
      int val = (depth - thresh0) / (thresh1 - thresh0) * 255.;
      return cv::Vec3b(val, val, 255 - val);
    }
    else if(depth >= thresh1 && depth < thresh2) {
      int val = (depth - thresh1) / (thresh2 - thresh1) * 255.;
      return cv::Vec3b(255, 255 - val, 0);
    }
    else if(depth >= thresh2 && depth < thresh3) {
      int val = (depth - thresh2) / (thresh3 - thresh2) * 255.;
      return cv::Vec3b(255 - val, val, 0);
    }
    
    return cv::Vec3b(0, 255, 0);
  }
  
  bool OniRecorder::toggleRecording()
  {
    recording_ = !recording_;
    
    cout << "Recording: " << recording_ << endl;
    if(recording_) {
      prev_depth_ts_ = numeric_limits<double>::quiet_NaN();
    }

    if(recording_ == false)
      return true;
    else
      return false;
  }
  
  void OniRecorder::initializeOpenNI()
  {
    ROS_ASSERT(mode_ == "VGA");
    
    model_.width_ = 640;
    model_.height_ = 480;
    model_.cx_ = model_.width_ / 2;
    model_.cy_ = model_.height_ / 2;

    // -- Set up production chain.
    XnStatus retval = XN_STATUS_OK;
    retval = context_.Init(); handleXnStatus(retval);
    retval = dgen_.Create(context_); handleXnStatus(retval);
  
    XnMapOutputMode output_mode;
    output_mode.nXRes = model_.width_;
    output_mode.nYRes = model_.height_;
    output_mode.nFPS = 30;
    retval = dgen_.SetMapOutputMode(output_mode); handleXnStatus(retval);


    retval = igen_.Create(context_); handleXnStatus(retval);
    retval = igen_.SetMapOutputMode(output_mode); handleXnStatus(retval);
    if(model_.type_ == "xpl") {
      retval = igen_.SetIntProperty("InputFormat", 5);  handleXnStatus(retval);  // Uncompressed YUV?  PCL openni_device_primesense.cpp:62.
      retval = igen_.SetPixelFormat(XN_PIXEL_FORMAT_YUV422); handleXnStatus(retval);
    }
    else if(model_.type_ == "kinect") {
      retval = igen_.SetIntProperty("InputFormat", 6);  handleXnStatus(retval);  // Some special Kinect image mode?
      retval = igen_.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT); handleXnStatus(retval);
    }
    
    // Synchronize output.
    if(model_.type_ != "kinect") {
      retval = dgen_.GetFrameSyncCap().FrameSyncWith(igen_); handleXnStatus(retval);
      ROS_ASSERT(dgen_.GetFrameSyncCap().IsFrameSyncedWith(igen_));
      ROS_ASSERT(igen_.GetFrameSyncCap().IsFrameSyncedWith(dgen_));
      ROS_DEBUG_STREAM("Using FrameSync.");
    }
    else
      ROS_DEBUG_STREAM("Not using FrameSync.");
    
    // Hardware depth registration.
    // https://groups.google.com/forum/?fromgroups=#!topic/openni-dev/5rP0mdPBeq0
    if(registered_) {
      cout << "Registering depth and rgb data." << endl;
      if(model_.type_ == "kinect") {
	retval = dgen_.SetIntProperty("RegistrationType", 2); handleXnStatus(retval);
      }
      else {
	retval = dgen_.SetIntProperty("RegistrationType", 1); handleXnStatus(retval);
      }
      
      retval = dgen_.GetAlternativeViewPointCap().SetViewPoint(igen_); handleXnStatus(retval);
      //retval = igen_.GetAlternativeViewPointCap().SetViewPoint(dgen_); handleXnStatus(retval);  // This fails.
    }
    else
      ROS_DEBUG_STREAM("Leaving depth and rgb unregistered.");


    // -- Add recording node.
    xn::Recorder recorder;
    retval = recorder.Create(context_); ROS_ASSERT(retval == XN_STATUS_OK);
    retval = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, "recording.oni"); ROS_ASSERT(retval == XN_STATUS_OK);
    //retval = recorder.AddNodeToRecording(dgen, XN_CODEC_16Z_EMB_TABLES); ROS_ASSERT(retval == XN_STATUS_OK);
    retval = recorder.AddNodeToRecording(dgen_); ROS_ASSERT(retval == XN_STATUS_OK);
    retval = recorder.AddNodeToRecording(igen_); ROS_ASSERT(retval == XN_STATUS_OK);
        
    retval = context_.StartGeneratingAll(); handleXnStatus(retval);
    
    // -- Set intrinsics.
    if(registered_) {
      model_.fx_ = 525;
      model_.fy_ = 525;
      ROS_DEBUG_STREAM("In registered mode.  Using hard-coded focal length of " << model_.fx_ << flush);
    }
    else {
      XnFieldOfView fov;
      dgen_.GetFieldOfView(fov);
      model_.fx_ = (double)model_.width_ / (2.0 * tan(fov.fHFOV / 2.0));
      model_.fy_ = (double)model_.height_ / (2.0 * tan(fov.fVFOV / 2.0));
      ROS_ASSERT(fabs(model_.fx_ - model_.fy_) < 1e-6);
      ROS_DEBUG_STREAM("Using focal length from device of " << model_.fx_ << flush); 
    }
  }
}


