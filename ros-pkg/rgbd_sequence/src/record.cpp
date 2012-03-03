#include <rgbd_sequence/recorder.h>

using namespace std;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: record [MODE]" << endl;
  oss << "  where MODE is VGA or QQVGA (default)." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  pcl::OpenNIGrabber::Mode mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz;
  if(argc == 2) {
    if(strcmp("VGA", argv[1]) == 0)
      mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;
    else if(strcmp("QQVGA", argv[1]) == 0)
      mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz;
    else { 
      cout << usageString() << endl;
      return 0;
    }
  }

  Recorder rec("", mode);
  rec.run();
  
  return 0;
}
