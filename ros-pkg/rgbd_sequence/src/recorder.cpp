#include <rgbd_sequence/recorder.h>
#include <bag_of_tricks/high_res_timer.h>

#define SHOW_IR (getenv("SHOW_IR"))
#define DESYNC (getenv("DESYNC"))
using namespace std;
namespace bfs = boost::filesystem;

namespace rgbd
{

  Recorder::Recorder(const std::string& device_id,
		     pcl::OpenNIGrabber::Mode mode) :
    device_id_(device_id),
    mode_(mode),
    grabber_(device_id_, mode, mode),
    cloud_viewer_("PointCloud"+device_id_),
    recording_(false)
  {
    initializeGrabber();
    clouds_.reserve(100000);
    imgs_.reserve(100000);
    image_timestamps_.reserve(100000);
    cloud_callback_timestamps_.reserve(100000);
  }
  
  void Recorder::run()
  {
    grabber_.start();
    while(!cloud_viewer_.wasStopped(1)) {
      usleep(1e3);
    }
    grabber_.stop();
  }

  void Recorder::cloudCallback(const Cloud::ConstPtr& cloud)
  {    
    if(recording_) {
      timespec clk;
      clock_gettime(CLOCK_REALTIME, &clk);
      cloud_callback_timestamps_.push_back(clk.tv_sec + clk.tv_nsec * 1e-9);
      
      clouds_.push_back(cloud);
    }
    else {
      cloud_viewer_.showCloud(cloud);
    }
  }

  cv::Mat1b Recorder::irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const
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
  
  cv::Mat3b Recorder::oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const
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
  
  void Recorder::imageCallback(const boost::shared_ptr<openni_wrapper::Image>& oni_img)
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
    //std::cout << "focal length is: " << grabber_.getDevice()->getImageFocalLength( img.cols ) << std::endl;
  }

  void Recorder::irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img)
  {
    if(!recording_) { 
      cv::Mat1b img = irToCV(oni_img);
      cv::namedWindow("IR", CV_WINDOW_NORMAL);
      cv::imshow("IR", img);
      cv::waitKey(3);
    }
  }

  void Recorder::depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni)
  {
    cout << "Depth timestamp: " << oni->getTimeStamp() << endl;
  }
  
  void Recorder::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
  {
    // if(event.getKeyCode())
    //   cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
    // else
    //   cout << "the special key \'" << event.getKeySym() << "\' was";
    // if(event.keyDown())
    //   cout << " pressed" << endl;
    // else
    //   cout << " released" << endl;

    if(event.getKeyCode() == 32 && event.keyDown()) {
      toggleRecording();
    }
  }

  std::string generateFilename(const bfs::path& dir,
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
  
  void Recorder::toggleRecording()
  {
    recording_ = !recording_;
    cout << "Recording: " << recording_ << endl;
    if(recording_)
      return;

    // string name;
    // cout << "Save sequence as: ";
    // cin >> name;
    string name = generateFilename("recorded_sequences", "seq", 4);
    
    
    // -- Get matching frames, put into Sequence.
    Sequence seq;
    size_t img_idx = 0;
    size_t pcd_idx = 0;
    // 30fps, so 1/60 sec is max possible difference.  Allow for 10% slop.
    double thresh = 1.1 * (1.0 / 60.0);
    double total_dt = 0;
    vector<double> filtered_cct;
    while(true) {
      double img_ts = image_timestamps_[img_idx];
      double pcd_ts = clouds_[pcd_idx]->header.stamp.toSec();
      double dt = fabs(img_ts - pcd_ts);
      cout << img_ts << " " << pcd_ts << ", |dt| = " << dt << endl;
      
      if(dt < thresh) {
	Cloud::Ptr tmp(new Cloud(*clouds_[pcd_idx]));
	filtered_cct.push_back(cloud_callback_timestamps_[pcd_idx]);
	seq.pcds_.push_back(tmp);

	if(mode_ == pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz) { 
	  // QQVGA appears to work for the pcd but not for the img.
	  // Resize the image here.
	  ROS_ASSERT(imgs_[img_idx].cols == 320);
	  ROS_ASSERT(imgs_[img_idx].rows == 240);
	  ROS_ASSERT(tmp->width == 160);
	  ROS_ASSERT(tmp->height == 120);
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
	  ROS_ASSERT(clouds_[pcd_idx]->width == 640);
	  ROS_ASSERT(clouds_[pcd_idx]->height == 480);
	  
	  seq.imgs_.push_back(imgs_[img_idx]);
	}

	++img_idx;
	++pcd_idx;
	total_dt += dt;
	cout << "Added." << endl;
      }
      else if(img_ts < pcd_ts)
	++img_idx;
      else
	++pcd_idx;

      if(img_idx == imgs_.size() || pcd_idx == clouds_.size())
	break;
    }

    cout << "Num images: " << imgs_.size() << endl;
    cout << "Num pcds: " << clouds_.size() << endl;
    cout << "Mean dt: " << total_dt / (double)seq.imgs_.size() << endl;
    cloud_callback_timestamps_ = filtered_cct;

    // -- Adjust the timestamps so that they reflect something close to system time.
    ROS_ASSERT(seq.pcds_.size() == cloud_callback_timestamps_.size());
    double mean_offset = 0;
    for(size_t i = 0; i < seq.pcds_.size(); ++i) { 
      mean_offset += cloud_callback_timestamps_[i] - seq.pcds_[i]->header.stamp.toSec();
    }
    mean_offset /= (double)seq.pcds_.size();
    cout << "Mean offset between sensor time and system time is " << setprecision(16) << mean_offset << endl;
    	
    for(size_t i = 0; i < seq.pcds_.size(); ++i) {
      double t = mean_offset + seq.pcds_[i]->header.stamp.toSec();
      seq.pcds_[i]->header.stamp.fromSec(t);
      cout << "Adjusted timestamp " << i << ": " << seq.pcds_[i]->header.stamp.fromSec(t) << endl;
    }

    // -- Save
    seq.save(name);
    cout << "Saved to " << name << endl;
    imgs_.clear();
    clouds_.clear();
    image_timestamps_.clear();
  }
  
  void Recorder::initializeGrabber()
  {
    cloud_viewer_.registerKeyboardCallback(&Recorder::keyboardCallback, *this, NULL);
    
    if(SHOW_IR) { 
      boost::function<void (const boost::shared_ptr<openni_wrapper::IRImage>&)> ir_cb;
      ir_cb = boost::bind(&Recorder::irCallback, this, _1);
      grabber_.registerCallback(ir_cb);
    }
    else {
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> image_cb;
      image_cb = boost::bind(&Recorder::imageCallback, this, _1);
      grabber_.registerCallback(image_cb);

      boost::function<void (const Cloud::ConstPtr&)> cloud_cb;
      cloud_cb = boost::bind(&Recorder::cloudCallback, this, _1);
      grabber_.registerCallback(cloud_cb);
      grabber_.getDevice()->setSynchronization(true);
      ROS_ASSERT(grabber_.getDevice()->isSynchronized());

      if(DESYNC)
	grabber_.getDevice()->setSynchronization(false);

      // boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> depth_cb;
      // depth_cb = boost::bind(&Recorder::depthImageCallback, this, _1);
      // grabber_.registerCallback(depth_cb);
    }
  }

}
