#include <signal.h>
#include <boost/program_options.hpp>
#include <openni2_interface/openni2_interface.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class OniHandlerExample : OpenNI2Handler
{
public:
  OpenNI2Interface oni_;

  OniHandlerExample(OpenNI2Interface::Resolution color_res,
                    OpenNI2Interface::Resolution depth_res) :
    oni_(color_res, depth_res)
  {
  }
  
  void run()
  {
    oni_.setHandler(this);
    oni_.run();
  }

  void rgbdCallback(const openni::VideoFrameRef& color,
                    const openni::VideoFrameRef& depth)
  {
    cout << "In OniHandlerExample::rgbdCallback." << endl;
    openni::DepthPixel* pDepth = (openni::DepthPixel*)depth.getData();
    int middleIndex = (depth.getHeight()+1)*depth.getWidth()/2;
    printf("[%08llu] color %d x %d; depth %d x %d; middle pixel depth:  %8d\n",
           (long long)depth.getTimestamp(),
           color.getWidth(), color.getHeight(),
           depth.getWidth(), depth.getHeight(),
           pDepth[middleIndex]);

    static int counter = 0;
    ++counter;
    if(counter > 100)
      oni_.terminate();
  }
  
};

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string color_resolution;
  string depth_resolution;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("color-res", bpo::value(&color_resolution), "")
    ("depth-res", bpo::value(&depth_resolution), "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  OpenNI2Interface::Resolution color_res = OpenNI2Interface::VGA;
  if(opts.count("color-res")) {
    if(color_resolution == "QVGA" || color_resolution == "qvga")
      color_res = OpenNI2Interface::QVGA;
    else if(color_resolution == "VGA" || color_resolution == "vga")
      color_res = OpenNI2Interface::VGA;
    else {
      cout << "Unrecognized resolution \"" << color_res << "\"." << endl;
      return 1;
    }
  }
  OpenNI2Interface::Resolution depth_res = OpenNI2Interface::VGA;
  if(opts.count("depth-res")) {
    if(depth_resolution == "QVGA" || depth_resolution == "qvga")
      depth_res = OpenNI2Interface::QVGA;
    else if(depth_resolution == "VGA" || depth_resolution == "vga")
      depth_res = OpenNI2Interface::VGA;
    else {
      cout << "Unrecognized resolution \"" << depth_res << "\"." << endl;
      return 1;
    }
  }

  OniHandlerExample ex(color_res, depth_res);
  ex.run();
  
  return 0;
}
