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

int main(int argc, char** argv)
{
  if(argc < 3) {
    cout << usageString() << endl;
    return 0;
  }

  double delay = atof(argv[1]);
  pcl::visualization::CloudViewer vis("Cloud");
  Cloud::Ptr pcd(new Cloud);
  for(int i = 2; i < argc; ++i) {
    pcl::io::loadPCDFile(argv[i], *pcd);
    cout << argv[i] << endl;
    vis.showCloud(pcd);
    usleep(delay * 1000);
  }
}
