#include <sentinel/sentinel.h>

using namespace std;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: record_stream MODE [IDX] " << endl;
  oss << "  where MODE is --vga (i.e. 640x480) or --qqvga (i.e. 160x120)." << endl;
  oss << "  where IDX is #1, #2, #3, etc" << endl;
  return oss.str();
}


int main(int argc, char** argv)
{
  if(argc != 2 && argc != 3) {
    cout << usageString() << endl;
    return -1;
  }

  pcl::OpenNIGrabber::Mode mode;
  if(strcmp("--vga", argv[1]) == 0)
    mode = pcl::OpenNIGrabber::OpenNI_VGA_30Hz;
  else if(strcmp("--qqvga", argv[1]) == 0)
    mode = pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz;
  else {
    cout << usageString() << endl;
    return 0;
  }

  string device_id = "";
  if(argc == 3)
    device_id = argv[2];

  double update_interval = 1;
  int max_training_imgs = 30;
  double threshold = 0.1;
  Sentinel sen(update_interval, max_training_imgs, threshold, device_id, mode);
  sen.run();

  return 0;
}
