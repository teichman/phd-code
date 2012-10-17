#include <dlvis/dlvis.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace rgbd;

int main(int argc, char** argv)
{
  DLVis vis;
  Cloud::Ptr all(new Cloud);
  for(int i = 1; i < argc; ++i) {
    cout << "Loading " << argv[i] << endl;
    Cloud pcd;
    pcl::io::loadPCDFile<Point>(argv[i], pcd);
    *all += pcd;
  }

  ThreadPtr thread = vis.launch();
  vis.setPointCloud(all);
  thread->join();
  return 0;
}
