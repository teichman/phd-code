#include <rgbd_sequence/mocap_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

int main(int argc, char** argv)
{
  TRCParser trc;
  trc.load(argv[1]);

  MocapVisualizer mv(trc, 0.005);
  mv.run();
  
  return 0;
}
