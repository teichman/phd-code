#include <boost/program_options.hpp>
#include <pcl/io/openni_grabber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rgbd_sequence/stream_recorder.h>

using namespace std;
using namespace rgbd;

bool g_debug = false;
double g_threshold = 1;
cv::Mat3b g_img;
cv::Mat3b g_background;

DepthMatPtr oniDepthToEigenPtr(const boost::shared_ptr<openni_wrapper::DepthImage>& oni)
{
  DepthMatPtr depth(new DepthMat(oni->getHeight(), oni->getWidth()));
  unsigned short data[depth->rows() * depth->cols()];
  oni->fillDepthImageRaw(depth->cols(), depth->rows(), data);
  int i = 0;
  for(int y = 0; y < depth->rows(); ++y){
    for(int x = 0; x < depth->cols(); ++x, ++i){
      if(data[i] == oni->getNoSampleValue() || data[i] == oni->getShadowValue()){
        depth->coeffRef(y,x) = 0;
      }
      else{
        depth->coeffRef(y,x) = data[i];
      }
    }
  }
  return depth;
}


cv::Mat3b oniToCV(const boost::shared_ptr<openni_wrapper::Image>& oni)
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


void rgbdCallback(const boost::shared_ptr<openni_wrapper::Image>& oni_rgb,
                  const boost::shared_ptr<openni_wrapper::DepthImage>& oni_depth,
                  float)
{
  DepthMatPtr depth = oniDepthToEigenPtr(oni_depth);
  g_img = oniToCV(oni_rgb);

  for(int y = 0; y < g_img.rows; ++y) {
    for(int x = 0; x < g_img.cols; ++x) {
      if(depth->coeffRef(y, x) == 0 ||
         depth->coeffRef(y, x) > g_threshold * 1000)
      {
        if(g_debug)
          g_img(y, x) = cv::Vec3b(0, 255, 0);
        else
          g_img(y, x) = g_background(y, x);
      }
    }
  }
  
  cv::imshow("img", g_img);
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("background", bpo::value<string>()->required(), "Background image to use")
    ;
  
  p.add("background", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: greenscreen BACKGROUND" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  g_background = cv::imread(opts["background"].as<string>(), 1);
  
  pcl::OpenNIGrabber::Mode mode(pcl::OpenNIGrabber::OpenNI_VGA_30Hz);
  pcl::OpenNIGrabber grabber("", mode, mode);

  boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&,
                  const boost::shared_ptr<openni_wrapper::DepthImage>&,
                  float)> rgbd_cb;
//rgbd_cb = boost::bind(&StreamRecorder::rgbdCallback, this, _1, _2, _3);
rgbd_cb = boost::bind(rgbdCallback, _1, _2, _3);
grabber.registerCallback(rgbd_cb);
//grabber.registerCallback(rgbdCallback);

  grabber.getDevice()->setSynchronization(true);
  ROS_ASSERT(grabber.getDevice()->isSynchronized());
  
  grabber.start();
  char key = ' ';
  while(true) {
    key = cv::waitKey(5);
    if(key == 'q')
      break;
    else if(key == '+')
      g_threshold += 0.2;
    else if(key == '-')
      g_threshold -= 0.2;
    else if(key == 'd')
      g_debug = !g_debug;
  }
  grabber.stop();

  return 0;
}
