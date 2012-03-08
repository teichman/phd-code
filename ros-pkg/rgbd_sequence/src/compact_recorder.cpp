#include <rgbd_sequence/compact_recorder.h>
#include <bag_of_tricks/high_res_timer.h>

#define SHOW_IR (getenv("SHOW_IR"))
#define DESYNC (getenv("DESYNC"))
using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  CompactRecorder::CompactRecorder(const std::string& device_id,
		     pcl::OpenNIGrabber::Mode mode) :
    device_id_(device_id),
    mode_(mode),
    grabber_(device_id_, mode, mode),
    cloud_viewer_("PointCloud"+device_id_),
    recording_(false)
  {
    initializeGrabber();
    depths_.reserve(100000);
    imgs_.reserve(100000);
    image_timestamps_.reserve(100000);
    depth_timestamps_.reserve(100000);
    depth_callback_timestamps_.reserve(100000);
  }
  
  void CompactRecorder::run()
  {
    grabber_.start();
    while(!cloud_viewer_.wasStopped(1)) {
      usleep(1e3);
    }
    grabber_.stop();
  }

  void CompactRecorder::cloudCallback(const Cloud::ConstPtr& cloud)
  {    
    if(recording_) {
      //PASS
    }
    else {
      cloud_viewer_.showCloud(cloud); //For now, so we can still visualize the cloud
    }
  }
  void CompactRecorder::depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni_depth_img)
  {
	  if(recording_) {
      timespec clk;
      clock_gettime(CLOCK_REALTIME, &clk);
      depth_callback_timestamps_.push_back(clk.tv_sec + clk.tv_nsec * 1e-9);
      depth_timestamps_.push_back((double)oni_depth_img->getTimeStamp() / (double)1e6);
	  	depths_.push_back( oniDepthToEigen(oni_depth_img) );
	  }
  }


  cv::Mat1b CompactRecorder::irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const
  {
    cv::Mat1b img(ir->getHeight(), ir->getWidth());
    unsigned short data[img.rows * img.cols];
    ir->fillRaw(img.cols, img.rows, data);
    int i = 0;
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x, ++i) {
	img(y, x) = data[i];
      }
    }
    
    return img;
  }
  
  cv::Mat3b CompactRecorder::oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const
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
  
  void CompactRecorder::imageCallback(const boost::shared_ptr<openni_wrapper::Image>& oni_img)
  {	
    cv::Mat3b img = oniToCV(oni_img);
    if(recording_) { 
      imgs_.push_back(img);
      image_timestamps_.push_back((double)oni_img->getTimeStamp() / (double)1e6);
    }
    else { 
      cv::imshow("Image"+device_id_, img);
      cv::waitKey(10);
    }
    std::cout << "focal length is: " << grabber_.getDevice()->getImageFocalLength( img.cols ) << std::endl;
  }

  void CompactRecorder::irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img)
  {
    if(!recording_) { 
      cv::Mat1b img = irToCV(oni_img);
      cv::namedWindow("IR", CV_WINDOW_NORMAL);
      cv::imshow("IR", img);
      cv::waitKey(3);
    }
  }

  void CompactRecorder::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
  {

    if(event.getKeyCode() == 32 && event.keyDown()) {
      toggleRecording();
    }
  }

  std::string generateFilenameCompact(const bfs::path& dir,
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
  
  void CompactRecorder::toggleRecording()
  {
    recording_ = !recording_;
    cout << "Recording: " << recording_ << endl;
    if(recording_)
      return;

    string name = generateFilenameCompact("recorded_sequences", "seq-"+device_id_, 4);
    
    
    // -- Get matching frames, put into Sequence.
    CompactSequence seq;
    seq.focal_length_ = focal_length_;
    size_t img_idx = 0;
    size_t dpt_idx = 0;
    // 30fps, so 1/60 sec is max possible difference.  Allow for 10% slop.
    double thresh = 1.1 * (1.0 / 60.0);
    double total_dt = 0;
    vector<double> filtered_cct;
    while(true) {
      double img_ts = image_timestamps_[img_idx];
      double pcd_ts = depth_timestamps_[dpt_idx];
      double dt = fabs(img_ts - pcd_ts);
      cout << img_ts << " " << pcd_ts << ", |dt| = " << dt << endl;
      
      if(dt < thresh) {
	filtered_cct.push_back(depth_callback_timestamps_[dpt_idx]);
    seq.depths_.push_back(depths_[dpt_idx]);
    seq.timestamps_.push_back(depth_callback_timestamps_[dpt_idx]);

	if(mode_ == pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz) { 
	  // QQVGA appears to work for the pcd but not for the img.
	  // Resize the image here.
	  ROS_ASSERT(imgs_[img_idx].cols == 320);
	  ROS_ASSERT(imgs_[img_idx].rows == 240);
	  ROS_ASSERT(depths_[dpt_idx].cols() == 160);
	  ROS_ASSERT(depths_[dpt_idx].rows() == 120);
	  // cout << "img size: " << imgs_[img_idx].cols << " " << imgs_[img_idx].rows << endl;
	  // cout << "pcd size: " << tmp->width << " " << tmp->height << endl;
	  
	  //seq.imgs_.push_back(imgs_[img_idx]);
	  cv::Mat3b small_img;
	  cv::resize(imgs_[img_idx], small_img, cv::Size(160, 120));
	  seq.imgs_.push_back(small_img);
	}
	else if(mode_ == pcl::OpenNIGrabber::OpenNI_VGA_30Hz) {
	  ROS_ASSERT(imgs_[img_idx].cols == 640);
	  ROS_ASSERT(imgs_[img_idx].rows == 480);
	  ROS_ASSERT(depths_[dpt_idx].cols() == 640);
	  ROS_ASSERT(depths_[dpt_idx].rows() == 480);
	  
	  seq.imgs_.push_back(imgs_[img_idx]);
	}

	++img_idx;
	++dpt_idx;
	total_dt += dt;
	cout << "Added." << endl;
      }
      else if(img_ts < pcd_ts)
	++img_idx;
      else
	++dpt_idx;

      if(img_idx == imgs_.size() || dpt_idx == depths_.size())
	break;
    }

    cout << "Num images: " << imgs_.size() << endl;
    cout << "Num depths: " << depths_.size() << endl;
    cout << "Mean dt: " << total_dt / (double)seq.imgs_.size() << endl;
    depth_callback_timestamps_ = filtered_cct;

    // -- Adjust the timestamps so that they reflect something close to system time.
    ROS_ASSERT(seq.timestamps_.size() == depth_callback_timestamps_.size());
    double mean_offset = 0;
    for(size_t i = 0; i < seq.depths_.size(); ++i) { 
      mean_offset += depth_callback_timestamps_[i] - seq.timestamps_[i];
    }
    mean_offset /= (double)seq.timestamps_.size();
    cout << "Mean offset between sensor time and system time is " << setprecision(16) << mean_offset << endl;
    	
    for(size_t i = 0; i < seq.timestamps_.size(); ++i) {
      double t = mean_offset + seq.timestamps_[i];
      seq.timestamps_[i] = t;
      cout << "Adjusted timestamp " << i << ": " << seq.timestamps_[i] << endl;
    }

    // -- Save
    seq.save(name);
    cout << "Saved to " << name << endl;
    imgs_.clear();
    depths_.clear();
    image_timestamps_.clear();
    depth_timestamps_.clear();
    depth_callback_timestamps_.clear();
  }
  
  void CompactRecorder::initializeGrabber()
  {
    cloud_viewer_.registerKeyboardCallback(&CompactRecorder::keyboardCallback, *this, NULL);
    
    if(SHOW_IR) { 
      boost::function<void (const boost::shared_ptr<openni_wrapper::IRImage>&)> ir_cb;
      ir_cb = boost::bind(&CompactRecorder::irCallback, this, _1);
      grabber_.registerCallback(ir_cb);
    }
    else {
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> image_cb;
      image_cb = boost::bind(&CompactRecorder::imageCallback, this, _1);
      grabber_.registerCallback(image_cb);

      boost::function<void (const Cloud::ConstPtr&)> cloud_cb;
      cloud_cb = boost::bind(&CompactRecorder::cloudCallback, this, _1);
      //grabber_.registerCallback(cloud_cb);

      boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> depth_cb;
      depth_cb = boost::bind(&CompactRecorder::depthImageCallback, this, _1);
      grabber_.registerCallback(depth_cb);

      grabber_.getDevice()->setSynchronization(true);
      ROS_ASSERT(grabber_.getDevice()->isSynchronized());

      if(DESYNC)
	grabber_.getDevice()->setSynchronization(false);

      // Get the focal length
      focal_length_ = grabber_.getDevice()->getImageFocalLength(img_width_);

    }
  }

  DepthMat CompactRecorder::oniDepthToEigen(const boost::shared_ptr<openni_wrapper::DepthImage>& oni) const
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

