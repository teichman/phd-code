#include <rgbd_sequence/stream_recorder.h>
#include <bag_of_tricks/high_res_timer.h>

#define SHOW_IR (getenv("SHOW_IR"))
#define DESYNC (getenv("DESYNC"))
using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  StreamRecorder::StreamRecorder(const std::string& device_id,
                                 pcl::OpenNIGrabber::Mode mode,
                                 const std::string& calib_file,
                                 bool view_cloud) :
    device_id_(device_id),
    mode_(mode),
    grabber_(device_id_, mode, mode),
    cloud_viewer_("PointCloud-" + device_id_),
    recording_(false),
    view_cloud_(view_cloud),
    calib_file_(calib_file),
    manual_calibration_(false)
  {
    initializeCalibration();
    initializeGrabber();
  }
  
  void StreamRecorder::run()
  {
    grabber_.start();
    while(!cloud_viewer_.wasStopped(1)) {
      usleep(1e3);
    }
    grabber_.stop();
  }

  void StreamRecorder::cloudCallback(const Cloud::ConstPtr& cloud)
  {    
    if(recording_) {
      //PASS
    }
    else {
      cloud_viewer_.showCloud(cloud); //For now, so we can still visualize the cloud
    }
  }

  void StreamRecorder::rgbdCallback(const boost::shared_ptr<openni_wrapper::Image>& oni_rgb,
                                    const boost::shared_ptr<openni_wrapper::DepthImage>& oni_depth, float f_inv)
  {
    if(recording_) {
      timespec clk;
      clock_gettime(CLOCK_REALTIME, &clk);
      double callback_timestamp = clk.tv_sec + clk.tv_nsec * 1e-9;
      double depth_timestamp = (double)oni_depth->getTimeStamp() / (double)1e6;
      double image_timestamp = (double)oni_rgb->getTimeStamp() / (double)1e6;
            
      double thresh = 1.1 * (1.0 / 60.0);
      if(fabs(depth_timestamp - image_timestamp) < thresh)
      {
        cout << " ********** Adding pair." << endl;
        cout << "rgbdCallback system timestamp: " << setprecision(16) << callback_timestamp << endl;
        cout << "depth system timestamp: " << setprecision(16) << oni_depth->getSystemTimeStamp() << endl;
        cout << "difference: " << setprecision(16) << oni_depth->getSystemTimeStamp() - callback_timestamp << endl;
        cout << "depth timestamp: " << setprecision(16) << depth_timestamp << endl;
        cout << "image timestamp: " << setprecision(16) << image_timestamp << endl;
        
        DepthMat depth = oniDepthToEigen( oni_depth );
        cv::Mat3b img = oniToCV(oni_rgb);
        seq_->addFrame( img, depth, fx_, fy_, cx_, cy_, oni_depth->getSystemTimeStamp() ); //TODO verify timing
      }
      else {
        ROS_WARN_STREAM("rgbdCallback got an rgbd pair with timestamp delta of " << depth_timestamp - image_timestamp);
      }
    }

    cv::Mat3b img = oniToCV(oni_rgb);
    cv::imshow("Image"+device_id_, img);
    cv::waitKey(10); // TODO: This is probably too long.
  }

  cv::Mat3b StreamRecorder::oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni)
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
  

  void StreamRecorder::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
  {

    if(event.getKeyCode() == 32 && event.keyDown()) {
      toggleRecording();
    }
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
    bfs::directory_iterator end_itr; // default construction yields past-the-end
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
  
  void StreamRecorder::toggleRecording()
  {
    recording_ = !recording_;
    cout << "Recording: " << recording_ << endl;
    if(recording_){
      string name = generateFilenameStream("recorded_sequences", "seq-"+device_id_, 4);
      seq_ = StreamSequence::Ptr( new StreamSequence( name ) );
    } else{
      //seq_.reset(); 
    }
  }
  
  void StreamRecorder::initializeGrabber()
  {
    cloud_viewer_.registerKeyboardCallback(&StreamRecorder::keyboardCallback, *this, NULL);
    
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&
                         ,const boost::shared_ptr<openni_wrapper::DepthImage>&
                         ,float)> rgbd_cb;
    rgbd_cb = boost::bind(&StreamRecorder::rgbdCallback, this, _1, _2, _3);
    grabber_.registerCallback(rgbd_cb);
      
    if(view_cloud_){
      boost::function<void (const Cloud::ConstPtr&)> cloud_cb;
      cloud_cb = boost::bind(&StreamRecorder::cloudCallback, this, _1);
      grabber_.registerCallback(cloud_cb);
    }

    grabber_.getDevice()->setSynchronization(true);
    ROS_ASSERT(grabber_.getDevice()->isSynchronized());

    if(DESYNC)
            grabber_.getDevice()->setSynchronization(false);
    if(!manual_calibration_){
      cout << "$XPL_CALIBRATION_FILE not set. Reverting to default calibration" << endl;
      fx_ = fy_ = grabber_.getDevice()->getImageFocalLength(image_width_);
      cx_ = (image_width_ >> 1);
      cy_ = (image_height_ >> 1);
    }

  }

  DepthMatPtr StreamRecorder::oniDepthToEigenPtr(const boost::shared_ptr<openni_wrapper::DepthImage>& oni)
  {
    DepthMatPtr depth(new DepthMat(oni->getHeight(), oni->getWidth()));
    unsigned short data[depth->rows() * depth->cols()];
    oni->fillDepthImageRaw(depth->cols(), depth->rows(), data);
    int i = 0;
    for(int y = 0; y < depth->rows(); ++y){
      for(int x = 0; x < depth->cols(); ++x, ++i){
        if(data[i] == oni->getNoSampleValue() || data[i] == oni->getShadowValue()){
          depth->coeffRef(y,x) = 0;
        }else{
          depth->coeffRef(y,x) = data[i];
        }
      }
    }
    return depth;
  }
  
  DepthMat StreamRecorder::oniDepthToEigen(const boost::shared_ptr<openni_wrapper::DepthImage>& oni)
  {
    return *oniDepthToEigenPtr(oni);
  }


  void StreamRecorder::initializeCalibration()
  {
    //First, get image size
    if(mode_ == pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz){
      image_width_ = 160;
      image_height_ = 120;
    }
    else if(mode_ == pcl::OpenNIGrabber::OpenNI_VGA_30Hz){
      image_width_ = 640;
      image_height_ = 480;
    }
    if (calib_file_ != ""){
      cout << "Using calibration file " << calib_file_ << endl;
      cv::FileStorage fs( calib_file_, cv::FileStorage::READ );
      // Get scale
      int calib_width, calib_height;
      fs["image_width"] >> calib_width;
      fs["image_height"] >> calib_height;
      float scale = image_width_ / float(calib_width);
      cv::Mat1f camera_matrix;
      fs["camera_matrix"] >> camera_matrix;
      // Scale the matrix, in pixels, appropriately
      camera_matrix *= scale;
      fx_ = camera_matrix(0,0);
      cx_ = camera_matrix(0,2);
      fy_ = camera_matrix(1,1);
      cy_ = camera_matrix(1,2);
      manual_calibration_ = true;
    }
  }
}


