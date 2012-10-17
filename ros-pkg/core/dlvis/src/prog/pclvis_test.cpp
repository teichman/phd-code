#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <rgbd_sequence/primesense_model.h>

using namespace std;
using namespace rgbd;

int main(int argc, char** argv)
{
  Cloud::Ptr all(new Cloud);
  for(int i = 1; i < argc; ++i) {
    cout << "Loading " << argv[i] << endl;
    Cloud pcd;
    pcl::io::loadPCDFile<Point>(argv[i], pcd);
    *all += pcd;
  }

  cout << "Visualizing " << all->size() << " points." << endl;
  pcl::visualization::CloudViewer viewer("pcd");
  viewer.showCloud(all);
  while(!viewer.wasStopped()) {
    usleep(1e3);
  }
      
  return 0;
}
