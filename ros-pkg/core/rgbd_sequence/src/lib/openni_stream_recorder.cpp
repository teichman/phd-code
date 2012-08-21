#include <rgbd_sequence/openni_stream_recorder.h>
#include <bag_of_tricks/high_res_timer.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  OpenNIStreamRecorder::OpenNIStreamRecorder(const std::string& mode) :
    mode_(mode),
    recording_(false),
    registered_(false),
    visualize_(true)
  {
    initializeOpenNI();
  }
  
  void OpenNIStreamRecorder::run()
  {
    double iters = 0;
    double mean_fps = std::numeric_limits<double>::quiet_NaN();
    double prev_ts = 0;
    cv::Mat1b dimg(cv::Size(width_, height_), 0);
    HighResTimer hrt;
    hrt.start();
    while(true) {
      char key = cv::waitKey(2);
      if(key == 'q')
	break;
      if(key == ' ')
	toggleRecording();

      //XnStatus retval = context_.WaitOneUpdateAll(dgen_);  // Apparently frame sync doesn't work when you do this.
      XnStatus retval = context_.WaitNoneUpdateAll();
      if(retval != XN_STATUS_OK) {
	printf("Failed updating data: %s\n", xnGetStatusString(retval));
	continue;
      }

      boost::shared_ptr<xn::DepthMetaData> dmd(new xn::DepthMetaData);
      dgen_.GetMetaData(*dmd);
      boost::shared_ptr<xn::ImageMetaData> imd(new xn::ImageMetaData);
      igen_.GetMetaData(*imd);
      ROS_ASSERT(imd->PixelFormat() == XN_PIXEL_FORMAT_YUV422);
    
      int width = dmd->GetUnderlying()->pMap->Res.X;
      int height = dmd->GetUnderlying()->pMap->Res.Y;
      ROS_ASSERT(width == (int)imd->GetUnderlying()->pMap->Res.X);
      ROS_ASSERT(height == (int)imd->GetUnderlying()->pMap->Res.Y);
      double depth_ts = dmd->Timestamp() * 1e-6;
      double image_ts = imd->Timestamp() * 1e-6;
      if(depth_ts == prev_ts)
	continue;
      if(fabs(depth_ts - image_ts) > 0.003)
	continue;
      if(depth_ts - prev_ts > 0.04)
	ROS_WARN("Dropping frames!");

      cout << "Width: " << width << ", height: " << height;
      cout << ", depth_ts: " << depth_ts << ", image_ts: " << image_ts;
      cout << ", depth_ts - image_ts: " << depth_ts - image_ts;
      cout << ", ts - prev_ts: " << depth_ts - prev_ts;
      cout << ", mean fps: " << mean_fps << endl;
      prev_ts = depth_ts;

      openni_wrapper::ImageYUV422 owimg(imd);
      // TODO: Get rid of these unnecessary openni_wrapper classes.
      cv::Mat3b cimg = oniToCV(owimg);
      ROS_ASSERT(fx_ == fy_);
      openni_wrapper::DepthImage owdimg(dmd, 0, fx_, 0, 0);
      if(recording_) {
	ScopedTimer st("Writing new frame");
	seq_->addFrame(cimg, *oniDepthToEigenPtr(owdimg), fx_, fy_, cx_, cy_, depth_ts);
      }
      
      if(visualize_) {
	for(int y = 0; y < dimg.rows; ++y) { 
	  for(int x = 0; x < dimg.cols; ++x) {
	    dimg(y, x) = 255.0 * (*dmd)(x, y) / 5000.0;  // dmd is in millimeters.
	    if(dimg(y, x) > 255.0)
	      dimg(y, x) = 0;
	  }
	}
	cv::imshow("Depth Image", dimg);
	cv::imshow("Color Image", cimg);
      }
      
      ++iters;
      mean_fps = iters / hrt.getSeconds();
    }
  }

  cv::Mat3b OpenNIStreamRecorder::oniToCV(const openni_wrapper::Image& oni)
  {
    cv::Mat3b img(oni.getHeight(), oni.getWidth());
    uchar data[img.rows * img.cols * 3];
    oni.fillRGB(img.cols, img.rows, data);
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

  std::string generateFilenameStream(const bfs::path& dir,
				     const std::string& basename,
				     int width)
  {
    // -- Create the directory if necessary.
    ROS_ASSERT(!bfs::exists(dir) || bfs::is_directory(dir));
    if(!bfs::exists(dir))
      bfs::create_directory(dir);

    // -- Find the next number.
    int num = 0;
    bfs::directory_iterator end_itr;  // default construction yields past-the-end
    for(bfs::directory_iterator itr(dir); itr != end_itr; ++itr) { 
      if(itr->leaf().substr(width+1).compare(basename) == 0)
	++num;
    }
    
    ostringstream filename;
    filename << setw(width) << setfill('0') << num << "-" << basename;
    ostringstream oss;
    oss << dir / filename.str();
    return oss.str();
  }
  
  void OpenNIStreamRecorder::toggleRecording()
  {
    recording_ = !recording_;
    cout << "Recording: " << recording_ << endl;
    if(recording_) {
      string name = generateFilenameStream("recorded_sequences", "seq", 5);
      seq_ = StreamSequence::Ptr(new StreamSequence(name));
    } 
  }
  
  DepthMatPtr OpenNIStreamRecorder::oniDepthToEigenPtr(const openni_wrapper::DepthImage& oni)
  {
    DepthMatPtr depth(new DepthMat(oni.getHeight(), oni.getWidth()));
    unsigned short data[depth->rows() * depth->cols()];
    oni.fillDepthImageRaw(depth->cols(), depth->rows(), data);
    int i = 0;
    for(int y = 0; y < depth->rows(); ++y){
      for(int x = 0; x < depth->cols(); ++x, ++i){
        if(data[i] == oni.getNoSampleValue() || data[i] == oni.getShadowValue()){
          depth->coeffRef(y,x) = 0;
        }else{
          depth->coeffRef(y,x) = data[i];
        }
      }
    }
    return depth;
  }
  
  DepthMat OpenNIStreamRecorder::oniDepthToEigen(const openni_wrapper::DepthImage& oni)
  {
    return *oniDepthToEigenPtr(oni);
  }

  void OpenNIStreamRecorder::initializeOpenNI()
  {
    ROS_ASSERT(mode_ == "VGA");
    
    width_ = 640;
    height_ = 480;

    // -- Set up production chain.
    XnStatus retval = XN_STATUS_OK;
    retval = context_.Init(); ROS_ASSERT(retval == XN_STATUS_OK);
    retval = dgen_.Create(context_); ROS_ASSERT(retval == XN_STATUS_OK);
    retval = igen_.Create(context_); ROS_ASSERT(retval == XN_STATUS_OK);
  
    XnMapOutputMode output_mode;
    output_mode.nXRes = width_;
    output_mode.nYRes = height_;
    output_mode.nFPS = 30;
    retval = dgen_.SetMapOutputMode(output_mode); ROS_ASSERT(retval == XN_STATUS_OK);
    retval = igen_.SetMapOutputMode(output_mode); ROS_ASSERT(retval == XN_STATUS_OK);
    retval = igen_.SetIntProperty("InputFormat", 5);  ROS_ASSERT(retval == XN_STATUS_OK);  // Uncompressed YUV?  PCL openni_device_primesense.cpp:62.
    retval = igen_.SetPixelFormat(XN_PIXEL_FORMAT_YUV422); ROS_ASSERT(retval == XN_STATUS_OK);
    // Hardware depth registration.
    // https://groups.google.com/forum/?fromgroups=#!topic/openni-dev/5rP0mdPBeq0
    if(registered_) {
      cout << "Registering depth and rgb data." << endl;
      retval = dgen_.SetIntProperty("RegistrationType", 1); ROS_ASSERT(retval == XN_STATUS_OK);  
      retval = dgen_.GetAlternativeViewPointCap().SetViewPoint(igen_); ROS_ASSERT(retval == XN_STATUS_OK);
      //retval = igen_.GetAlternativeViewPointCap().SetViewPoint(dgen_); ROS_ASSERT(retval == XN_STATUS_OK);  // This fails.
    }
    else
      cout << "Leaving depth and rgb unregistered." << endl;
  
    // Synchronize output.
    retval = dgen_.GetFrameSyncCap().FrameSyncWith(igen_); ROS_ASSERT(retval == XN_STATUS_OK);
    retval = context_.StartGeneratingAll(); ROS_ASSERT(retval == XN_STATUS_OK);
    
    // -- Set intrinsics.
    if(registered_) {
      fx_ = 525;
      fy_ = 525;
      ROS_DEBUG_STREAM("In registered mode.  Using hard-coded focal length of " << fx_ << flush);
    }
    else {
      XnFieldOfView fov;
      dgen_.GetFieldOfView(fov);
      fx_ = (double)width_ / (2.0 * tan(fov.fHFOV / 2.0));
      fy_ = (double)height_ / (2.0 * tan(fov.fVFOV / 2.0));
      ROS_ASSERT(fabs(fx_ - fy_) < 1e-6);
      ROS_DEBUG_STREAM("Using focal length from device of " << fx_ << flush); 
    }
  }
}


