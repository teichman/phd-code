#include <dst/recorder.h>
#include <bag_of_tricks/high_res_timer.h>

#define SHOW_IR (getenv("SHOW_IR"))
#define DESYNC (getenv("DESYNC"))
using namespace std;

namespace dst
{
  
  Recorder::Recorder(const std::string& device_id) :
    device_id_(device_id),
    grabber_(device_id_,
	     pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz,
	     pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz),
    cloud_viewer_("PointCloud"),
    recording_(false)
  {
    initializeGrabber();
    clouds_.reserve(100000);
    images_.reserve(100000);
    image_timestamps_.reserve(100000);
  }
  
  void Recorder::run()
  {
    grabber_.start();
    while(!cloud_viewer_.wasStopped(1)) {
      usleep(1e3);
    }
    grabber_.stop();
  }

  void Recorder::cloudCallback(const KinectCloud::ConstPtr& cloud)
  {
    //ScopedTimer st("cloud callback");
    
//    static double prev = 0;
    //double delta = cloud->header.stamp.toSec() - prev;
//    ROS_WARN_STREAM_COND(delta > 0.05, "Dropped at least one pointcloud.  Delta is " << delta << ".  (Frame alignment should not be affected.)");
//    prev = cloud->header.stamp.toSec();

    //lock();
    if(recording_) {
      clouds_.push_back(cloud);
    }
    else {
      cloud_viewer_.showCloud(cloud);
    }
    //unlock();
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
    //ScopedTimer st("image callback");
    //lock();
    cv::Mat3b img = oniToCV(oni_img);
    if(recording_) { 
      images_.push_back(img);
      image_timestamps_.push_back((double)oni_img->getTimeStamp() / (double)1e6);
    }
    else { 
      cv::imshow("Image", img);
      cv::waitKey(10);
    }
    //unlock();
  }

  void Recorder::irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img)
  {
    //ScopedTimer st("ir image callback");
    //lock();
    if(!recording_) { 
      cv::Mat1b img = irToCV(oni_img);
      cv::namedWindow("IR", CV_WINDOW_NORMAL);
      cv::imshow("IR", img);
      cv::waitKey(3);
    }
    //unlock();
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

  void Recorder::toggleRecording()
  {
    recording_ = !recording_;
    cout << "Recording: " << recording_ << endl;
    if(recording_)
      return;

    string name;
    cout << "Save sequence as: ";
    cin >> name;

    // -- Get matching frames, put into KinectSequence.
    KinectSequence seq;
    size_t img_idx = 0;
    size_t pcd_idx = 0;
    // 30fps, so 1/60 sec is max possible difference.  Allow for 10% slop.
    double thresh = 1.1 * (1.0 / 60.0);
    double total_dt = 0;
    while(true) {
      double img_ts = image_timestamps_[img_idx];
      double pcd_ts = clouds_[pcd_idx]->header.stamp.toSec();
      double dt = fabs(img_ts - pcd_ts);
      cout << img_ts << " " << pcd_ts << ", |dt| = " << dt << endl;
      
      if(dt < thresh) {
	KinectCloud::Ptr tmp(new KinectCloud(*clouds_[pcd_idx]));
	seq.pointclouds_.push_back(tmp);

	// QQVGA appears to work for the pcd but not for the img.
	// Resize the image here.
	ROS_ASSERT(images_[img_idx].cols == 320);
	ROS_ASSERT(images_[img_idx].rows == 240);
	ROS_ASSERT(tmp->width == 160);
	ROS_ASSERT(tmp->height == 120);
	// cout << "img size: " << images_[img_idx].cols << " " << images_[img_idx].rows << endl;
	// cout << "pcd size: " << tmp->width << " " << tmp->height << endl;
	
	//seq.images_.push_back(images_[img_idx]);
	cv::Mat3b small_img;
	cv::resize(images_[img_idx], small_img, cv::Size(160, 120));
	seq.images_.push_back(small_img);

	++img_idx;
	++pcd_idx;
	total_dt += dt;
	cout << "Added." << endl;
      }
      else if(img_ts < pcd_ts)
	++img_idx;
      else
	++pcd_idx;

      if(img_idx == images_.size() || pcd_idx == clouds_.size())
	break;
    }

    cout << "Num images: " << images_.size() << endl;
    cout << "Num pcds: " << clouds_.size() << endl;
    cout << "Mean dt: " << total_dt / (double)seq.images_.size() << endl;

    seq.save(name);
    cout << "Saved to " << name << endl;
    images_.clear();
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

      boost::function<void (const KinectCloud::ConstPtr&)> cloud_cb;
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
