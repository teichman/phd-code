#include <rgbd_sequence/openni_stream_recorder.h>
#include <bag_of_tricks/high_res_timer.h>

using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  OpenNIStreamRecorder::OpenNIStreamRecorder(const std::string& type,
					     int id,
					     const std::string& mode,
					     bool fake_rgb,
					     bool registered) :
    sequences_dir_("recorded_sequences"),
    mode_(mode),
    recording_(false),
    fake_rgb_(fake_rgb),
    registered_(registered),
    visualize_(true),
    prev_depth_ts_(numeric_limits<double>::quiet_NaN()),
    sync_(0.02)  // Set for the kinect.
  {
    ROS_ASSERT(type == "kinect" || type == "xpl");
    ROS_ASSERT(!(fake_rgb && registered));  // This setting would make no sense.
    model_.type_ = type;
    model_.id_ = id;
    initializeOpenNI();
  }
  
  void OpenNIStreamRecorder::run()
  {
    //ProfilerStart("OpenNIStreamRecorder.prof");
    
    while(true) {
      char key = cv::waitKey(2);
      if(key == 'q')
	break;
      if(key == ' ')
	toggleRecording();

      if(fake_rgb_) 
	getDepth();
      else
	getRGBD();
    }

    //ProfilerStop();
  }

  void OpenNIStreamRecorder::handleXnStatus(const XnStatus& status) const
  {
    if(status != XN_STATUS_OK) {
      ROS_FATAL_STREAM("OpenNI failure:" << endl << xnGetStatusString(status));
      ROS_ASSERT(status == XN_STATUS_OK);
    }
  }
      
  void OpenNIStreamRecorder::getDepth()
  {
    XnStatus retval = context_.WaitNoneUpdateAll();
    if(retval != XN_STATUS_OK) {
      printf("Failed updating data: %s\n", xnGetStatusString(retval));
      return;
    }
    
    DMDPtr dmd(new xn::DepthMetaData);
    dgen_.GetMetaData(*dmd);
    double depth_ts = dmd->Timestamp() * 1e-6;

    // -- Force it to use every frame.
    if(sync_.ts0_ != depth_ts) {
      sync_.addT0(dmd, depth_ts);
      sync_.addT1(IMDPtr(), depth_ts);
    }
    processSynchronizedData();
  }

  void OpenNIStreamRecorder::getRGBD()
  {
    XnStatus retval = context_.WaitNoneUpdateAll();
    if(retval != XN_STATUS_OK) {
      printf("Failed updating data: %s\n", xnGetStatusString(retval));
      return;
    }
    
    boost::shared_ptr<xn::DepthMetaData> dmd(new xn::DepthMetaData);
    dgen_.GetMetaData(*dmd);
    double depth_ts = dmd->Timestamp() * 1e-6;

    boost::shared_ptr<xn::ImageMetaData> imd(new xn::ImageMetaData);
    igen_.GetMetaData(*imd);
    if(model_.type_ == "xpl")
      ROS_ASSERT(imd->PixelFormat() == XN_PIXEL_FORMAT_YUV422);
    double image_ts = imd->Timestamp() * 1e-6;

    if(sync_.mostRecent0() != depth_ts) {
      //cout << "=== Adding depth_ts: " << depth_ts << endl;
      sync_.addT0(dmd, depth_ts);
    }
    processSynchronizedData();
    
    if(sync_.mostRecent1() != image_ts) {
      //cout << "=== Adding image_ts: " << image_ts << endl;
      sync_.addT1(imd, image_ts);
    }
    processSynchronizedData();    
  }

  cv::Vec3b OpenNIStreamRecorder::colorize(double depth, double min_range, double max_range) const
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
  
  void OpenNIStreamRecorder::processSynchronizedData()
  {
    if(!sync_.updated_)
      return;
    sync_.updated_ = false;

    //cout << "=== sync_ has updated.  depth_ts: " << sync_.ts0_ << ", image_ts: " << sync_.ts1_ << endl;
    
    if(sync_.ts0_ - prev_depth_ts_ > 0.04)
      ROS_WARN("Dropping frames!");

    // -- Fill the frame with depth data.
    Frame frame;
   //ROS_DEBUG_STREAM("fx: " << model_.fx_ << ", fy: " << model_.fy_);
    ROS_ASSERT(fabs(model_.fx_ - model_.fy_) < 1e-6);
    openni_wrapper::DepthImage owdimg(sync_.current0_, 0, model_.fx_, 0, 0);
    frame.depth_ = oniDepthToEigenPtr(owdimg);
    frame.timestamp_ = sync_.ts0_;

    // -- Print out information.
    int width = sync_.current0_->GetUnderlying()->pMap->Res.X;
    int height = sync_.current0_->GetUnderlying()->pMap->Res.Y;

    if(recording_) {
      cout << "Width: " << setw(3) << width << ", height: " << setw(3) << height;
      cout << ", depth_ts: " << fixed << setw(7) << setprecision(4) << sync_.ts0_;
      cout << ", image_ts: " << fixed << setw(7) << setprecision(4) << sync_.ts1_;
      cout << ", depth_ts - image_ts: " << fixed << setw(7) << setprecision(4) << sync_.ts0_ - sync_.ts1_;
      cout << ", ts - prev_depth_ts_: " << fixed << setw(7) << setprecision(4) << sync_.ts0_ - prev_depth_ts_;
      cout << endl;
    }
    
    prev_depth_ts_ = sync_.ts0_;

    // -- Get the rgb data.
    cv::Mat3b cimg(cv::Size(width, height));
    double min_depth = 0.1;
    double max_depth = 10.0;
    if(fake_rgb_) {
      for(int y = 0; y < cimg.rows; ++y)
	for(int x = 0; x < cimg.cols; ++x)
	  cimg(y, x) = colorize(frame.depth_->coeffRef(y, x) / 1000.0, min_depth, max_depth);
    }
    else {
      ROS_ASSERT(width == (int)sync_.current1_->GetUnderlying()->pMap->Res.X);
      ROS_ASSERT(height == (int)sync_.current1_->GetUnderlying()->pMap->Res.Y);
      // TODO: Get rid of these unnecessary openni_wrapper classes.
      HighResTimer hrt2("Dealing with openni_wrapper");
      hrt2.start();
      if(model_.type_ == "kinect") {
	openni_wrapper::ImageBayerGRBG owimg(sync_.current1_, openni_wrapper::ImageBayerGRBG::EdgeAwareWeighted);
	cimg = oniToCV(owimg);
      }
      else {
	openni_wrapper::ImageYUV422 owimg(sync_.current1_);
	cimg = oniToCV(owimg);
      }
    }
    frame.img_ = cimg;
    
    if(recording_) {
      //ScopedTimer st("Writing new frame");
      seq_->writeFrame(frame);
    }

    if(visualize_) {
      cv::Mat3b dimg(cv::Size(width, height));
      double min_depth = 0.1;
      double max_depth = 12.0;
      for(int y = 0; y < dimg.rows; ++y)
	for(int x = 0; x < dimg.cols; ++x)
	  dimg(y, x) = colorize(frame.depth_->coeffRef(y, x) * 0.001, min_depth, max_depth);
      
      cv::imshow("Depth Image", dimg);
      cv::imshow("Color Image", frame.img_);
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
    bfs::directory_iterator end_itr;  // default construction yields past-the-end
    vector<string> dirs;
    for(bfs::directory_iterator itr(dir); itr != end_itr; ++itr) {
      if(itr->leaf().substr(0, basename.size()).compare(basename) == 0)
	dirs.push_back(itr->leaf());
    }

    sort(dirs.begin(), dirs.end());

    // -- Choose the next number, filling holes.
    size_t idx;
    for(idx = 0; idx < dirs.size(); ++idx) {
      string numstr = dirs[idx].substr(basename.size() + 4, basename.size() + 7);
      int num = atoi(numstr.c_str());
      ROS_ASSERT(num >= 0);
      if(idx < (size_t)num)
	break;
    }
    
    ostringstream filename;
    filename << basename << "-seq" << setw(width) << setfill('0') << idx;
    ostringstream oss;
    oss << dir / filename.str();
    return oss.str();
  }
  
  void OpenNIStreamRecorder::toggleRecording()
  {
    recording_ = !recording_;
    
    cout << "Recording: " << recording_ << endl;
    if(recording_) {
      prev_depth_ts_ = numeric_limits<double>::quiet_NaN();
			  
      string name = generateFilenameStream(sequences_dir_, model_.name(), 3);
      cout << "Saving to " << name << endl;
      seq_ = StreamSequence::Ptr(new StreamSequence);
      seq_->init(name);
      seq_->model_ = model_;
      seq_->save();
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

    if(fake_rgb_) {
      ROS_DEBUG_STREAM("Not recording rgb data.");
    }
    else {
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
    }
  
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


