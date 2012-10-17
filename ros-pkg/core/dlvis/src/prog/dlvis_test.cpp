#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <xpl_calibration/object_matching_calibrator.h>
#include <dlvis/dlvis.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

void keyboardCallback(unsigned char c, int x, int y)
{
  cout << "Got keypress: " << c << endl;
}

int main(int argc, char** argv)
{
  Cloud::Ptr all(new Cloud);
  bool ps = false;
  int i = 1;
  if(string(argv[1]) == "--primesense") {
    ps = true;
    ++i;
  }
  for(; i < argc; ++i) {
    cout << "Loading " << argv[i] << endl;
    Cloud pcd;
    pcl::io::loadPCDFile<Point>(argv[i], pcd);
    if(ps) {
      Affine3f transform = generateTransform(- M_PI / 2.0, 0.0, -M_PI / 2.0, 0, 0, 0);
      pcl::transformPointCloud(pcd, pcd, transform);
    }
    *all += pcd;
  }

  DLVis vis;
  vis.registerKeyboardCallback(keyboardCallback);
  ThreadPtr thread = vis.launch();
  vis.setPointCloud(all);

  while(true) {
    usleep(1e6);
    // do something else, update visualizer, etc.
  }
  
  thread->join();
  return 0;
}
