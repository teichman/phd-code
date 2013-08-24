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

  OniHandlerExample(OpenNI2Interface::Resolution resolution) : oni_(resolution) {}
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
    printf("[%08llu] %d x %d;  %8d\n", (long long)depth.getTimestamp(), depth.getWidth(), depth.getHeight(), pDepth[middleIndex]);

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

  string resolution;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("resolution,r", bpo::value(&resolution), "")
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

  OpenNI2Interface::Resolution res = OpenNI2Interface::VGA;
  if(opts.count("resolution")) {
    if(resolution == "QVGA" || resolution == "qvga")
      res = OpenNI2Interface::QVGA;
    else {
      cout << "Unrecognized resolution \"" << resolution << "\"." << endl;
      return 1;
    }
  }

  OniHandlerExample ex(res);
  ex.run();
  
  return 0;
}
