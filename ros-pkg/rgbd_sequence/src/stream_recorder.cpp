#include <rgbd_sequence/stream_recorder.h>
#include <bag_of_tricks/high_res_timer.h>

#define SHOW_IR (getenv("SHOW_IR"))
#define DESYNC (getenv("DESYNC"))
using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  StreamRecorder::StreamRecorder(const std::string& device_id,
		     pcl::OpenNIGrabber::Mode mode) :
    device_id_(device_id),
    mode_(mode),
    grabber_(device_id_, mode, mode),
    cloud_viewer_("PointCloud"+device_id_),
    recording_(false)
  {
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
  void rgbdCallback( const boost::shared_ptr<openni_wrapper::Image>& oni_rgb,
      const boost::shared_ptr<openni_wrapper::DepthImage>& oni_depth, float f_inv )
  {
    cv::Mat3b img = oniToCV(oni_img);
	  if(recording_) {
      timespec clk;
      clock_gettime(CLOCK_REALTIME, &clk);
      double real_timesamp = clk.tv_sec + clk.tv_nsec * 1e-9;
      double depth_timestamp = (double)oni_depth_img->getTimeStamp() / (double)1e6;
      double image_timestamps = (double)oni_img->getTimeStamp() / (double)1e6;
      double thresh = 1.1 * (1.0 / 60.0);
      if(fabs(depth_timestamp - image_timestamp) < thresh)
      {
        DepthMat depth = oniDepthtoEigen( oni_rgb );
        seq_->addFrame( img, depth, 1.0/f_inv, real_timestamp ); //TODO verify timing
      }
	  }
    else{
      cv::imshow("Image"+device_id_, img);
      cv::waitKey(10);
    }
  }

  cv::Mat3b StreamRecorder::oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const
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
      seq_.reset(); 
    }
  }
  
  void StreamRecorder::initializeGrabber()
  {
    cloud_viewer_.registerKeyboardCallback(&StreamRecorder::keyboardCallback, *this, NULL);
    
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&
                         ,const boost::shared_ptr<openni_wrapper::DepthImage>&
                         ,float)> rgbd_cb;
    rgbd_cb = boost::bind(&StrreamRecorder::rgbdCallback, this, _1, _2, _3);
    grabber_.registerCallback(rgbd_cb);
      
    boost::function<void (const Cloud::ConstPtr&)> cloud_cb;
    cloud_cb = boost::bind(&CompactRecorder::cloudCallback, this, _1);
    grabber_.registerCallback(cloud_cb);

    grabber_.getDevice()->setSynchronization(true);
    ROS_ASSERT(grabber_.getDevice()->isSynchronized());

    if(DESYNC)
	    grabber_.getDevice()->setSynchronization(false);
  }

  DepthMat StreamRecorder::oniDepthToEigen(const boost::shared_ptr<openni_wrapper::DepthImage>& oni) const
  {
    DepthMat depth(oni->getHeight(), oni->getWidth());
    unsigned short data[depth.rows() * depth.cols()];
    oni->fillDepthImageRaw(depth.cols(), depth.rows(), data);
    int i = 0;
    for(int y = 0; y < depth.rows(); ++y){
      for(int x = 0; x < depth.cols(); ++x, ++i){
        if(data[i] == oni->getNoSampleValue() || data[i] == oni->getShadowValue()){
          depth(y,x) = 0;
        }else{
          depth(y,x) = data[i];
        }
      }
    }
  return depth;
  }

}


