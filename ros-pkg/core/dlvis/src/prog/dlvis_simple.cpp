#include <pcl/io/pcd_io.h>
#include <xpl_calibration/object_matching_calibrator.h>
#include <dlvis/dlvis.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

int main(int argc, char** argv)
{
  Cloud::Ptr pcd(new Cloud);
  pcl::io::loadPCDFile<Point>(argv[1], *pcd);

  DLVis vis;
  ThreadPtr thread = vis.launch();
  vis.setPointCloud(pcd);
  thread->join();
  return 0;
}
