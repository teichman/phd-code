#include <rgbd_sequence/rgbd_sequence.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: pcd_series_view DELAY PCD [PCD ...]" << endl;
  return oss.str();
}

bool g_pressed = false;
void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.getKeyCode() == 32 && event.keyDown())
    g_pressed = true;
}

int main(int argc, char** argv)
{
  if(argc < 3) {
    cout << usageString() << endl;
    return 0;
  }

  int delay = atoi(argv[1]);
  pcl::visualization::CloudViewer vis("Cloud");
  if(delay == 0) { 
    vis.registerKeyboardCallback(&keyboardCallback, NULL);
    cout << "No delay set.  Press space to advance." << endl;
  }
  
  Cloud::Ptr pcd(new Cloud);
  for(int i = 2; i < argc; ++i) {
    pcl::io::loadPCDFile(argv[i], *pcd);
    cout << argv[i] << endl;
    vis.showCloud(pcd);
    if(delay)
      usleep(delay * 1000);
    else { 
      while(true) {
	if(g_pressed) {
	  g_pressed = false;
	  break;
	}
	usleep(1000);
      }
    }
  }
}
