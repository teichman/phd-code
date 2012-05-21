#include <pcl/io/openni_grabber.h>
#include <image_labeler/opencv_view.h>
#include <bag_of_tricks/lockable.h>

using namespace std;

class Inspector : public OpenCVViewDelegate, public Lockable
{
public:
  void run();
  Inspector();
  
protected:
  cv::Mat1d dmap_;
  cv::Mat3b vis_;
  OpenCVView view_;

  void mouseEvent(int event, int x, int y, int flags, void* param);
  void callback(const boost::shared_ptr<openni_wrapper::DepthImage>&);
  void updateDepth(const openni_wrapper::DepthImage& oni);
  std::vector< boost::shared_ptr<openni_wrapper::DepthImage> > buffer_;
};

Inspector::Inspector() :
  view_("Depth image")
{
  view_.setDelegate(this);
}

void Inspector::mouseEvent(int event, int x, int y, int flags, void* param)
{
  lock();
  if(event == CV_EVENT_LBUTTONDOWN) {
    cout << "Click at " << x << " " << y << ".  Depth: " << dmap_(y, x) << endl;
  }
  unlock();
}

void Inspector::run()
{
  pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;
  cv::Size sz(640, 480);
  dmap_ = cv::Mat1d(sz);
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
    default:
      break;
    }
  }
}

void Inspector::updateDepth(const openni_wrapper::DepthImage& oni)
{
  dmap_ = 0;
  ushort data[oni.getHeight() * oni.getWidth()];
  oni.fillDepthImageRaw(oni.getWidth(), oni.getHeight(), data);
  int i = 0;
  for(size_t y = 0; y < oni.getHeight(); ++y) {
    for(size_t x = 0; x < oni.getWidth(); ++x, ++i) {
      if(data[i] == oni.getNoSampleValue() || data[i] == oni.getShadowValue())
	continue;
      dmap_(y, x) = data[i] * 0.001f;
    }
  }

  vis_ = cv::Vec3b(0, 0, 0);
  for(int y = 0; y < dmap_.rows; ++y) {
    for(int x = 0; x < dmap_.cols; ++x) {
      if(dmap_(y, x) == 0)
	continue;
      
      double maxdist = 10;
      double mindist = 1;
      double val = 255 * (1.0 - ((fmin(maxdist, fmax(mindist, dmap_(y, x))) - mindist) / (maxdist - mindist)));
      vis_(y, x) = cv::Vec3b(val, val, val);
    }
  }
}

void Inspector::callback(const boost::shared_ptr<openni_wrapper::DepthImage>& oni)
{
  lock();
  buffer_.push_back(oni);
  unlock();
}

int main(int argc, char** argv)
{
  Inspector inspector;
  inspector.run();
  return 0;
}
