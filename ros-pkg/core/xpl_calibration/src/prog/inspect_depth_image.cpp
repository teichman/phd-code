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
  cv::Mat3b vis_;
  OpenCVView view_;
  bool use_intrinsics_;

  void mouseEvent(int event, int x, int y, int flags, void* param);
  void callback(const boost::shared_ptr<openni_wrapper::DepthImage>&);
  void updateDepth(const openni_wrapper::DepthImage& oni);
  std::vector< boost::shared_ptr<openni_wrapper::DepthImage> > buffer_;
};

Inspector::Inspector() :
  dddm_(NULL),
  show_pcd_(false),
  view_("Depth image"),
  use_intrinsics_(false)
{
  view_.setDelegate(this);
  frame_.depth_ = DepthMatPtr(new DepthMat(480, 640));
  frame_.depth_->setZero();
}

void Inspector::mouseEvent(int event, int x, int y, int flags, void* param)
{
  lock();
  if(event == CV_EVENT_LBUTTONDOWN) {
    cout << "Click at " << x << " " << y << ".  Depth: " << frame_.depth_->coeffRef(y, x) * 0.001 << endl;
  }
  unlock();
}

void Inspector::run()
{
  pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;
  cv::Size sz(640, 480);
  vis_ = cv::Mat3b(sz, cv::Vec3b(0, 0, 0));
  
  pcl::OpenNIGrabber grabber("", mode, mode);
  boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> cb;
  cb = boost::bind(&Inspector::callback, this, _1);
  grabber.registerCallback(cb);
  grabber.start();

  bool done = false;
  while(!done) {
    lock();
    if(!buffer_.empty()) {
      view_.updateImage(vis_);
      updateDepth(*buffer_.back());
    }
    buffer_.clear();
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

void Inspector::updateDepth(const openni_wrapper::DepthImage& oni)
{
  frame_.depth_->setZero();
  ushort data[oni.getHeight() * oni.getWidth()];
  oni.fillDepthImageRaw(oni.getWidth(), oni.getHeight(), data);
  int i = 0;
  for(size_t y = 0; y < oni.getHeight(); ++y) {
    for(size_t x = 0; x < oni.getWidth(); ++x, ++i) {
      if(data[i] == oni.getNoSampleValue() || data[i] == oni.getShadowValue())
	continue;
      frame_.depth_->coeffRef(y, x) = data[i];
    }
  }

  if(dddm_ && use_intrinsics_)
    dddm_->undistort(&frame_);
  
  vis_ = frame_.depthImage();
  frame_.img_ = vis_.clone();

  static pcl::visualization::CloudViewer viewer("pcd");

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
    viewer.showCloud(cloud);
  }
  
  // vis_ = cv::Vec3b(0, 0, 0);
  // for(int y = 0; y < frame_.depth_->rows(); ++y) {
  //   for(int x = 0; x < frame_.depth_->cols(); ++x) {
  //     if(frame_.depth_->coeffRef(y, x) == 0)
  // 	continue;
      
  //     double maxdist = 10;
  //     double mindist = 1;
  //     double val = 255 * (1.0 - ((fmin(maxdist, fmax(mindist, frame_.depth_->coeffRef(y, x))) - mindist) / (maxdist - mindist)));
  //     vis_(y, x) = cv::Vec3b(val, val, val);
  //   }
  // }
}

void Inspector::callback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni)
{
  lock();
  buffer_.push_back(oni);
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
