#include <boost/program_options.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <image_labeler/opencv_view.h>
#include <bag_of_tricks/lockable.h>
#include <xpl_calibration/discrete_depth_distortion_model.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

class Inspector : public OpenCVViewDelegate, public Lockable
{
public:
  DiscreteDepthDistortionModel* dddm_;
  bool show_pcd_;
  
  void run();
  Inspector();
  
protected:
  Frame frame_;
  cv::Mat3b img_vis_;
  OpenCVView view_;
  bool use_intrinsics_;
  pcl::visualization::PCLVisualizer pcd_vis_;
  std::vector< boost::shared_ptr<openni_wrapper::Image> > image_buffer_;
  std::vector< boost::shared_ptr<openni_wrapper::DepthImage> > depth_buffer_;

  void mouseEvent(int event, int x, int y, int flags, void* param);
  cv::Mat3b oniToCV(const openni_wrapper::Image& oni) const;
  void idiCallback(const boost::shared_ptr<openni_wrapper::Image>& img,
                   const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
                   float callback);
  void updateDepth(const openni_wrapper::Image& img,
                   const openni_wrapper::DepthImage& depth);
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
};

Inspector::Inspector() :
  dddm_(NULL),
  show_pcd_(false),
  view_("Depth image"),
  use_intrinsics_(false),
  pcd_vis_("Cloud")
{
  view_.setDelegate(this);
  frame_.depth_ = DepthMatPtr(new DepthMat(480, 640));
  frame_.depth_->setZero();

  pcd_vis_.addCoordinateSystem(0.3);
  pcd_vis_.setBackgroundColor(0, 0, 0);
  Cloud::Ptr cloud(new Cloud);
  pcd_vis_.addPointCloud(cloud, "cloud");
  pcd_vis_.registerKeyboardCallback(&Inspector::keyboardCallback, *this);
}

void Inspector::mouseEvent(int event, int x, int y, int flags, void* param)
{
  lock();
  if(event == CV_EVENT_LBUTTONDOWN) {
    cout << "Click at " << x << " " << y << ".  Depth: " << frame_.depth_->coeffRef(y, x) * 0.001 << endl;
  }
  unlock();
}

void Inspector::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.keyDown() && event.getKeyCode() == 'm') {
    use_intrinsics_ = !use_intrinsics_;
    cout << "use_intrinsics_: " << use_intrinsics_ << endl;
  }
}

void Inspector::run()
{
  pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;
  cv::Size sz(640, 480);
  img_vis_ = cv::Mat3b(sz, cv::Vec3b(0, 0, 0));
  
  pcl::OpenNIGrabber grabber("", mode, mode);
  // -- Register a callback that just gets the depth image.
  // boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> cb;
  // cb = boost::bind(&Inspector::callback, this, _1);
  // grabber.registerCallback(cb);

  // -- Register a callback for both the depth image and the camera image.
  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&,
                        const boost::shared_ptr<openni_wrapper::DepthImage>&,
                        float)> cb;
  cb = boost::bind(&Inspector::idiCallback, this, _1, _2, _3);
  grabber.registerCallback(cb);
  
  grabber.start();

  bool done = false;
  while(!done) {
    lock();
    if(!image_buffer_.empty()) {
      view_.updateImage(img_vis_);
      updateDepth(*image_buffer_.back(), *depth_buffer_.back());
    }
    image_buffer_.clear();
    depth_buffer_.clear();
    unlock();
    char key = view_.cvWaitKey(10);
    switch(key) {
    case 'q':
      done = true;
      break;
    case 'r':
      grabber.getDevice()->setDepthRegistration(!grabber.getDevice()->isDepthRegistered());
      cout << "Depth - RGB registration: " << grabber.getDevice()->isDepthRegistered() << endl;
      break;
    case 'm':
      use_intrinsics_ = !use_intrinsics_;
      cout << "use_intrinsics_: " << use_intrinsics_ << endl;
      break;
    default:
      break;
    }
  }
}

cv::Mat3b Inspector::oniToCV(const openni_wrapper::Image& oni) const
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

void Inspector::updateDepth(const openni_wrapper::Image& image,
                            const openni_wrapper::DepthImage& depth)
{
  frame_.depth_->setZero();
  ushort data[depth.getHeight() * depth.getWidth()];
  depth.fillDepthImageRaw(depth.getWidth(), depth.getHeight(), data);
  int i = 0;
  for(size_t y = 0; y < depth.getHeight(); ++y) {
    for(size_t x = 0; x < depth.getWidth(); ++x, ++i) {
      if(data[i] == depth.getNoSampleValue() || data[i] == depth.getShadowValue())
        continue;
      frame_.depth_->coeffRef(y, x) = data[i];
    }
  }

  if(dddm_ && use_intrinsics_)
    dddm_->undistort(&frame_);

  // Use the actual rgb data from the sensor.
  frame_.img_ = oniToCV(image);
  
  // something else
  //frame_.img_ = img_vis_.clone();

  // Don't show colors at all.
  // static cv::Mat3b blank(cv::Size(depth.getWidth(), depth.getHeight()), cv::Vec3b(255, 255, 255));
  // frame_.img_ = blank;
  
  if(show_pcd_) {
    Cloud::Ptr cloud(new Cloud);
    PrimeSenseModel model;
    model.width_ = 640;
    model.height_ = 480;
    model.cx_ = model.width_ / 2;
    model.cy_ = model.height_ / 2;
    model.fx_ = 525;
    model.fy_ = 525;
    model.frameToCloud(frame_, cloud.get());
    pcd_vis_.updatePointCloud(cloud, "cloud");
    pcd_vis_.spinOnce(2);
  }  
}

void Inspector::idiCallback(const boost::shared_ptr<openni_wrapper::Image>& img,
                            const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
                            float constant)
{
  lock();
  image_buffer_.push_back(img);
  depth_buffer_.push_back(depth);
  unlock();
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string sequence_path;
  string trajectory_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("intrinsics", bpo::value<string>(), "Optional discrete distortion model.")
    ("pcd,p", "Show pointcloud.")
    ;

  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " OPTS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  int retval = system("killall XnSensorServer"); --retval;
  
  Inspector inspector;
  if(opts.count("intrinsics")) {
    inspector.dddm_ = new DiscreteDepthDistortionModel;
    inspector.dddm_->load(opts["intrinsics"].as<string>());
    cout << "Loading DiscreteDepthDistortionModel at " << opts["intrinsics"].as<string>() << endl;
  }
  if(opts.count("pcd"))
    inspector.show_pcd_ = true;
  
  inspector.run();

  if(inspector.dddm_)
    delete inspector.dddm_;
  
  return 0;
}
