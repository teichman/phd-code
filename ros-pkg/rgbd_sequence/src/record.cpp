#include <rgbd_sequence/recorder.h>

using namespace std;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: record MODE" << endl;
  oss << "  where MODE is --vga (i.e. 640x480) or --qqvga (i.e. 160x120)." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }
  
  pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz;
  if(strcmp("--vga", argv[1]) == 0)
    mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;
  else if(strcmp("--qqvga", argv[1]) == 0)
    mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz;
  else {
    cout << usageString() << endl;
    return 0;
  }
  
  Recorder rec("", mode);
  rec.run();
  
  return 0;
}
