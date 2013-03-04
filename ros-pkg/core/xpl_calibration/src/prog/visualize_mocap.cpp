#include <xpl_calibration/mocap_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: visualize_mocap TRC SSEQ" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return -1;
  }
    
  TRCParser trc;
  trc.load(argv[1]);
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory(argv[2]);
  MocapVisualizer mv(trc, sseq);
  mv.run();
  
  return 0;
}
