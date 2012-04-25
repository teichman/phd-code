#include <multi_sequence/multi_recorder.h>

using namespace std;
using namespace multi_sequence;

string usageString()
{
  ostringstream oss;
  oss << "Usage: record MODE IDX1 IDX2 ..." << endl;
  oss << "  where MODE is --vga (i.e. 640x480) or --qqvga (i.e. 160x120)." << endl;
  oss << "  where IDX is #1, #2, #3, etc" << endl;
  oss << "  Timestamps are based on system time at callback." << endl;
  oss << "  If recording on multiple machines, you probably want to run 'sudo ntpdate -b ntp.ubuntu.com' on both beforehand." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 3) {
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
  vector<string> device_ids;
  for(int i = 0; i < argc-2; i++){
    device_ids.push_back( argv[i+2] );
  }
  MultiRecorder rec(device_ids, mode);
  rec.run();
  
  return 0;
}


