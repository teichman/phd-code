#include <rgbd_sequence/mocap_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

int main(int argc, char** argv)
{
  TRCParser trc;
  trc.load(argv[1]);

  // Fabricate xpl data for testing.
  vector<Cloud::Ptr> xpl;
  for(size_t i = 0; i < trc.frames_.size(); ++i) {
    Cloud::Ptr pcd(new Cloud);
    *pcd = *trc.frames_[i];
    xpl.push_back(pcd);
  }
  
  MocapVisualizer mv(trc, xpl, 0.0075);
  mv.run();
  
  return 0;
}
