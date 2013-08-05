#include <sentinel/sentinel.h>

using namespace std;

string usageString()
{
  ostringstream oss;
  oss << "Usage: record_stream MODE NAME [IDX] " << endl;
  oss << "  MODE is --vga (i.e. 640x480) or --qvga (i.e. 320x240)." << endl;
  oss << "  NAME is a string that describes this location." << endl;
  oss << "  IDX is #1, #2, #3, etc" << endl;
  return oss.str();
}


int main(int argc, char** argv)
{
  if(argc != 3 && argc != 4) {
    cout << usageString() << endl;
    return -1;
  }

  pcl::OpenNIGrabber::Mode mode;
  if(strcmp("--vga", argv[1]) == 0)
    mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;
  else if(strcmp("--qvga", argv[1]) == 0)
    mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz; // OpenNIGrabber bug.
  else {
    cout << usageString() << endl;
    return 0;
  }

  string name = argv[2];
  string device_id = "";
  if(argc == 4)
    device_id = argv[3];
  
  double update_interval = 1;
  double save_interval = 1;
  double threshold = 0.00001;
  int max_training_imgs = 180;
  Sentinel sen(name, update_interval, save_interval, max_training_imgs, threshold, device_id, mode);
  sen.run();

  return 0;
}
