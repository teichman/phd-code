#include <openni2_interface/openni2_interface.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class OniHandlerExample : OpenNI2Handler
{
public:
  OpenNI2Interface oni_;
  
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
  }
  
};

int main(int argc, char** argv)
{
  OniHandlerExample ex;
  ex.run();
  
  return 0;
}
