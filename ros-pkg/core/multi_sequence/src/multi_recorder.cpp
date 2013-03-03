#include <multi_sequence/multi_recorder.h>
#include <multi_sequence/multi_sequence.h>
#include <bag_of_tricks/high_res_timer.h>

#define SHOW_IR (getenv("SHOW_IR"))
#define DESYNC (getenv("DESYNC"))
using namespace std;
namespace bfs = boost::filesystem;

namespace multi_sequence
{

  MultiRecorder::MultiRecorder(const vector<string> &device_ids,
                     pcl::OpenNIGrabber::Mode mode) :
    device_ids_(device_ids),
    mode_(mode),
    recording_(false)
  {
    clouds_.resize(device_ids_.size());
    imgs_.resize(device_ids_.size());
    image_timestamps_.resize(device_ids_.size());
    for(size_t i = 0; i < device_ids_.size(); i++){
      grabbers_.push_back(new pcl::OpenNIGrabber(device_ids_[i], mode, mode ) );
      cloud_viewers_.push_back(new pcl::visualization::CloudViewer(device_ids_[i]));
      clouds_[i].reserve(100000);
      imgs_[i].reserve(100000);
      image_timestamps_[i].reserve(100000);
      initializeGrabber(i);
    }
  }
  
  void MultiRecorder::run()
  {
    for(size_t i = 0; i < grabbers_.size(); i++){
      grabbers_[i]->start();
    }
    bool stopped = false;
    while(!stopped) {
      usleep(1e3);
      for(size_t i = 0; i < cloud_viewers_.size(); i++){
        stopped |= cloud_viewers_[i]->wasStopped(1);
      }
    }
    for(size_t i = 0; i < grabbers_.size(); i++){
      grabbers_[i]->stop();
    }
  }

  void MultiRecorder::cloudCallback(const Cloud::ConstPtr& cloud, size_t idx)
  {
    if(recording_) {
      clouds_[idx].push_back(cloud);
    }
    else {
      cloud_viewers_[idx]->showCloud(cloud);
    }
  }

  cv::Mat1b MultiRecorder::irToCV(const boost::shared_ptr<openni_wrapper::IRImage>& ir) const
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
  
  cv::Mat3b MultiRecorder::oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni) const
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
  
  void MultiRecorder::imageCallback(const boost::shared_ptr<openni_wrapper::Image>& oni_img, size_t idx)
  {
    cv::Mat3b img = oniToCV(oni_img);
    if(recording_) { 
      imgs_[idx].push_back(img);
      image_timestamps_[idx].push_back((double)oni_img->getTimeStamp() / (double)1e6);
    }
    else { 
      cv::imshow("Image"+device_ids_[idx], img);
      cv::waitKey(10);
    }
  }

  void MultiRecorder::irCallback(const boost::shared_ptr<openni_wrapper::IRImage>& oni_img)
  {
    if(!recording_) { 
      cv::Mat1b img = irToCV(oni_img);
      cv::namedWindow("IR", CV_WINDOW_NORMAL);
      cv::imshow("IR", img);
      cv::waitKey(3);
    }
  }

  void MultiRecorder::depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni)
  {
    cout << "Depth timestamp: " << oni->getTimeStamp() << endl;
  }
  
  void MultiRecorder::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
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
  
  void MultiRecorder::toggleRecording()
  {
    recording_ = !recording_;
    cout << "Recording: " << recording_ << endl;
    if(recording_)
      return;

    vector<Sequence::Ptr> seqs(grabbers_.size());
    string name = generateFilename("recorded_sequences", "seq", 4);
    double thresh = 1.1 * (1.0 / 60.0);
    for(size_t i = 0; i < grabbers_.size(); i++){
      seqs[i] = Sequence::Ptr( new Sequence );
      // -- Get matching frames, put into Sequence.
      size_t img_idx = 0;
      size_t pcd_idx = 0;
      // 30fps, so 1/60 sec is max possible difference.  Allow for 10% slop.
      double total_dt = 0;
      while(true) {
        double img_ts = image_timestamps_[i][img_idx];
        double pcd_ts = clouds_[i][pcd_idx]->header.stamp.toSec();
        double dt = fabs(img_ts - pcd_ts);
        cout << img_ts << " " << pcd_ts << ", |dt| = " << dt << endl;
        
        if(dt < thresh) {
                Cloud::Ptr tmp(new Cloud(*clouds_[i][pcd_idx]));
                seqs[i]->pcds_.push_back(tmp);
                if(mode_ == pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz) { 
                  ROS_ASSERT(imgs_[i][img_idx].cols == 320);
                  ROS_ASSERT(imgs_[i][img_idx].rows == 240);
                  ROS_ASSERT(tmp->width == 160);
                  ROS_ASSERT(tmp->height == 120);
                  cv::Mat3b small_img;
                  cv::resize(imgs_[i][img_idx], small_img, cv::Size(160, 120));
                  seqs[i]->imgs_.push_back(small_img);
                }
                else if(mode_ == pcl::OpenNIGrabber::OpenNI_VGA_30Hz) {
                  ROS_ASSERT(imgs_[i][img_idx].cols == 640);
                  ROS_ASSERT(imgs_[i][img_idx].rows == 480);
                  ROS_ASSERT(clouds_[i][pcd_idx]->width == 640);
                  ROS_ASSERT(clouds_[i][pcd_idx]->height == 480);
                  
                  seqs[i]->imgs_.push_back(imgs_[i][img_idx]);
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

        if(img_idx == imgs_[i].size() || pcd_idx == clouds_[i].size()){
                break;
        }
      }
      imgs_[i].clear();
      clouds_[i].clear();
      image_timestamps_[i].clear();
    }
    //TODO fix this to work with StreamSequences
    //MultiSequence mseq(thresh, seqs);
    //mseq.save(name);
  }
  
  void MultiRecorder::initializeGrabber(size_t idx)
  {
    cloud_viewers_[idx]->registerKeyboardCallback(&MultiRecorder::keyboardCallback, *this, NULL);
    
    if(SHOW_IR) { 
      boost::function<void (const boost::shared_ptr<openni_wrapper::IRImage>&)> ir_cb;
      ir_cb = boost::bind(&MultiRecorder::irCallback, this, _1);
      grabbers_[idx]->registerCallback(ir_cb);
    }
    else {
      boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> image_cb;
      image_cb = boost::bind(&MultiRecorder::imageCallback, this, _1, idx);
      grabbers_[idx]->registerCallback(image_cb);

      boost::function<void (const Cloud::ConstPtr&)> cloud_cb;
      cloud_cb = boost::bind(&MultiRecorder::cloudCallback, this, _1, idx);
      grabbers_[idx]->registerCallback(cloud_cb);
      grabbers_[idx]->getDevice()->setSynchronization(true);
      ROS_ASSERT(grabbers_[idx]->getDevice()->isSynchronized());

      if(DESYNC)
              grabbers_[idx]->getDevice()->setSynchronization(false);

    }
  }

}

